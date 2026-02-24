#include "esc_openyge.h"

#include <math.h>
#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define TIMEOUT_US 2000
#define FRAME_START 0xA5
#define CRC_POLYNOMIAL 0x1021  // CRC-16-CCITT

// CRC mode tracking (based on your Arduino code)
static crc_mode_t g_crc_mode = CRC_MODE_UNKNOWN;

static uint16_t calculate_crc16_with_seed(uint8_t *data, uint8_t length, uint16_t seed);
static crc_mode_t detect_crc_mode(uint8_t *frame, uint8_t frame_len);
static bool validate_crc(uint8_t *frame, uint8_t frame_len, crc_mode_t mode);
static void process(esc_openyge_parameters_t *parameter);

void esc_openyge_task(void *parameters) {
    esc_openyge_parameters_t parameter = *(esc_openyge_parameters_t *)parameters;

    // Initialize all output variables
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature_fet = 0;
    *parameter.temperature_bec = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.voltage_bec = 0;
    *parameter.current_bec = 0;
    *parameter.throttle = 0;
    *parameter.pwm_percent = 0;
    *parameter.cell_count = 1;
    parameter.crc_errors = 0;
    parameter.crc_mode = CRC_MODE_UNKNOWN;

    xTaskNotifyGive(context.receiver_task_handle);

#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
    *parameter.consumption = 123.4;
    *parameter.voltage = 22.2;
    *parameter.current = 15.5;
    *parameter.temperature_fet = 45.6;
    *parameter.temperature_bec = 38.2;
    *parameter.cell_voltage = 3.7;
    *parameter.voltage_bec = 5.1;
    *parameter.current_bec = 0.8;
    *parameter.throttle = 75;
    *parameter.pwm_percent = 80;
#endif

    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);


    // Initialize UART for OpenYGE protocol (115200 baud, 8N1, inverted = false)
    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);

    debug("\nOpenYGE ESC init");

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_openyge_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    uint8_t length = uart1_available();

    if (length < 8) {  // Minimum frame size
        return;
    }

    uint8_t data[length];
    uart1_read_bytes(data, length);

    debug("\nOpenYGE (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(data, length, "0x%02X ");

    // Look for frame start in the received data
    for (int i = 0; i <= length - 8; i++) {  // Minimum 8 bytes needed
        if (data[i] == FRAME_START) {
            uint8_t *frame = &data[i];
            uint8_t frame_len = (i + 4 < length) ? frame[3] : 0;

            // Check if we have enough data for complete frame
            if (frame_len == 0 || (i + frame_len) > length || frame_len < 8) {
                continue;
            }

            // Validate frame structure - version 3, type 0
            if (frame[1] != 3 || frame[2] != 0 || frame[3] != frame_len) {
                continue;
            }

            // Auto-detect CRC mode if unknown
            if (parameter->crc_mode == CRC_MODE_UNKNOWN) {
                crc_mode_t detected_mode = detect_crc_mode(frame, frame_len);
                if (detected_mode != CRC_MODE_UNKNOWN) {
                    parameter->crc_mode = detected_mode;
                    debug("\nOpenYGE CRC mode auto-detected: %d", detected_mode);
                }
            }

            // Validate CRC using detected or known mode
            if (parameter->crc_mode != CRC_MODE_UNKNOWN && 
                validate_crc(frame, frame_len, parameter->crc_mode)) {
                // Parse telemetry data using correct byte positions from your working code

                // FET Temperature - byte 7, subtract 40 for actual temperature
                int temp_fet_raw = (int)frame[7] - 40;
                float temperature_fet = (temp_fet_raw < -40 || temp_fet_raw > 215) ? 0 : temp_fet_raw;

                // Voltage - bytes 8-9 (little endian), scale by 0.01 
                uint16_t voltage_raw = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8);
                float voltage = voltage_raw * 0.01f;  // V_SCALE from your code

                // Current - bytes 10-11 (little endian), scale by 0.01
                uint16_t current_raw = (uint16_t)frame[10] | ((uint16_t)frame[11] << 8);
                float current = current_raw * 0.01f;  // A_SCALE from your code

                // Consumption - bytes 12-13 (little endian), direct mAh
                uint16_t consumption_raw = (uint16_t)frame[12] | ((uint16_t)frame[13] << 8);

                // eRPM - bytes 14-15 (little endian), scale by 0.1, then apply pole pairs
                uint16_t erpm_raw = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8);
                float erpm = erpm_raw * 0.1f;  // ERPM_SCALE from your code
                float rpm = (erpm / 2.0f) * parameter->rpm_multiplier;  // Assuming 2 pole pairs like your code

                // PWM - byte 16, direct percentage
                float pwm_percent = frame[16];

                // Throttle - byte 17, direct percentage  
                float throttle = frame[17];

                // BEC voltage - bytes 18-19 (little endian), in mV
                uint16_t bec_voltage_raw = (uint16_t)frame[18] | ((uint16_t)frame[19] << 8);
                float voltage_bec = bec_voltage_raw / 1000.0f;

                // BEC current - bytes 20-21 (little endian), in mA
                uint16_t bec_current_raw = (uint16_t)frame[20] | ((uint16_t)frame[21] << 8);
                float current_bec = bec_current_raw / 1000.0f;

                // BEC/Cap Temperature - byte 22 or 24, subtract 40 for actual temperature
                int temp_bec_raw = (int)frame[22] - 40;  // Try byte 22 first
                float temperature_bec = (temp_bec_raw < -40 || temp_bec_raw > 215) ? 0 : temp_bec_raw;

                // Update outputs with averaging
                if (parameter->pwm_out) xTaskNotifyGive(context.pwm_out_task_handle);

                *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
                *parameter->consumption += get_consumption(*parameter->current, 65535, &timestamp);
                *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
                *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
                *parameter->temperature_fet = get_average(parameter->alpha_temperature, *parameter->temperature_fet, temperature_fet);
                *parameter->temperature_bec = get_average(parameter->alpha_temperature, *parameter->temperature_bec, temperature_bec);
                *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
                *parameter->voltage_bec = voltage_bec;
                *parameter->current_bec = current_bec;
                *parameter->throttle = throttle;
                *parameter->pwm_percent = pwm_percent;

                debug(
                    "\nOpenYGE (%u) < RPM: %.0f Volt: %.2fV Curr: %.2fA TempFET: %.1f°C "
                    "TempBEC: %.1f°C Cons: %.0fmAh CellV: %.2fV BECVolt: %.2fV BECCurr: %.3fA Thr: %.0f%% PWM: %.0f%%",
                    uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, 
                    *parameter->current, *parameter->temperature_fet, *parameter->temperature_bec, *parameter->consumption,
                    *parameter->cell_voltage, *parameter->voltage_bec, *parameter->current_bec, *parameter->throttle, *parameter->pwm_percent);

                return; // Successfully processed frame
            } else {
                parameter->crc_errors++;

                if (parameter->crc_mode == CRC_MODE_UNKNOWN) {
                    debug("\nOpenYGE CRC mode unknown, trying to detect...");
                } else {
                    debug("\nOpenYGE CRC error with mode %d", parameter->crc_mode);
                }

                // Debug frame contents for first few errors
                if (parameter->crc_errors <= 3) {
                    debug(" Frame[%d]: ", frame_len);
                    for (int j = 0; j < frame_len && j < 12; j++) {
                        debug("%02X ", frame[j]);
                    }
                }
            }
        }
    }
}

static uint16_t calculate_crc16_with_seed(uint8_t *data, uint8_t length, uint16_t seed) {
    uint16_t crc = seed;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static crc_mode_t detect_crc_mode(uint8_t *frame, uint8_t frame_len) {
    if (frame_len < 4) return CRC_MODE_UNKNOWN;

    // Extract CRC from frame in both byte orders
    uint16_t rx_be = ((uint16_t)frame[frame_len-2] << 8) | frame[frame_len-1];  // Big-endian
    uint16_t rx_le = ((uint16_t)frame[frame_len-1] << 8) | frame[frame_len-2];  // Little-endian

    // Try all CRC variants
    struct {
        crc_mode_t mode;
        uint8_t start_offset;
        uint16_t seed;
        bool is_big_endian;
    } variants[] = {
        {CRC_MODE_INC_SYNC_SEED_FFFF_BE,  0, 0xFFFF, true},
        {CRC_MODE_INC_SYNC_SEED_FFFF_LE,  0, 0xFFFF, false},
        {CRC_MODE_SKIP_SYNC_SEED_FFFF_BE, 1, 0xFFFF, true},
        {CRC_MODE_SKIP_SYNC_SEED_FFFF_LE, 1, 0xFFFF, false},
        {CRC_MODE_INC_SYNC_SEED_0000_BE,  0, 0x0000, true},
        {CRC_MODE_INC_SYNC_SEED_0000_LE,  0, 0x0000, false},
        {CRC_MODE_SKIP_SYNC_SEED_0000_BE, 1, 0x0000, true},
        {CRC_MODE_SKIP_SYNC_SEED_0000_LE, 1, 0x0000, false},
    };

    for (int i = 0; i < 8; i++) {
        if (frame_len <= (2 + variants[i].start_offset)) continue;

        uint16_t calc_crc = calculate_crc16_with_seed(
            frame + variants[i].start_offset, 
            frame_len - 2 - variants[i].start_offset, 
            variants[i].seed
        );

        uint16_t expected_crc = variants[i].is_big_endian ? rx_be : rx_le;

        if (calc_crc == expected_crc) {
            debug("\nOpenYGE CRC mode detected: %d", variants[i].mode);
            return variants[i].mode;
        }
    }

    return CRC_MODE_UNKNOWN;
}

static bool validate_crc(uint8_t *frame, uint8_t frame_len, crc_mode_t mode) {
    if (frame_len < 4) return false;

    uint16_t rx_be = ((uint16_t)frame[frame_len-2] << 8) | frame[frame_len-1];
    uint16_t rx_le = ((uint16_t)frame[frame_len-1] << 8) | frame[frame_len-2];

    uint8_t start_offset;
    uint16_t seed;
    bool is_big_endian;

    switch (mode) {
        case CRC_MODE_INC_SYNC_SEED_FFFF_BE:  start_offset = 0; seed = 0xFFFF; is_big_endian = true; break;
        case CRC_MODE_INC_SYNC_SEED_FFFF_LE:  start_offset = 0; seed = 0xFFFF; is_big_endian = false; break;
        case CRC_MODE_SKIP_SYNC_SEED_FFFF_BE: start_offset = 1; seed = 0xFFFF; is_big_endian = true; break;
        case CRC_MODE_SKIP_SYNC_SEED_FFFF_LE: start_offset = 1; seed = 0xFFFF; is_big_endian = false; break;
        case CRC_MODE_INC_SYNC_SEED_0000_BE:  start_offset = 0; seed = 0x0000; is_big_endian = true; break;
        case CRC_MODE_INC_SYNC_SEED_0000_LE:  start_offset = 0; seed = 0x0000; is_big_endian = false; break;
        case CRC_MODE_SKIP_SYNC_SEED_0000_BE: start_offset = 1; seed = 0x0000; is_big_endian = true; break;
        case CRC_MODE_SKIP_SYNC_SEED_0000_LE: start_offset = 1; seed = 0x0000; is_big_endian = false; break;
        default: return false;
    }

    uint16_t calc_crc = calculate_crc16_with_seed(
        frame + start_offset, 
        frame_len - 2 - start_offset, 
        seed
    );

    uint16_t expected_crc = is_big_endian ? rx_be : rx_le;
    return calc_crc == expected_crc;
}

// Temperature function no longer needed - OpenYGE sends direct Celsius values