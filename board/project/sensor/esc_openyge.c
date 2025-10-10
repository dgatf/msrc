#include "esc_openyge.h"

#include <math.h>
#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define TIMEOUT_US 2000
#define PACKET_LENGTH 34
#define FRAME_START 0xA5
#define FRAME_VERSION 3
#define FRAME_TYPE 0
#define FRAME_LENGTH 34
#define CRC_POLYNOMIAL 0x1021  // CRC-16-CCITT

static uint16_t calculate_crc16(uint8_t *data, uint8_t length);
static void process(esc_openyge_parameters_t *parameter);
static float get_temperature_celsius(uint16_t raw_temp);

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
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

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
    
    if (length < PACKET_LENGTH) {
        return;
    }

    uint8_t data[length];
    uart1_read_bytes(data, length);

    debug("\nOpenYGE (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(data, length, "0x%02X ");

    // Look for frame start in the received data
    for (int i = 0; i <= length - PACKET_LENGTH; i++) {
        if (data[i] == FRAME_START && (i + PACKET_LENGTH <= length)) {
            uint8_t *frame = &data[i];
            
            // Validate frame structure
            if (frame[1] == FRAME_VERSION && frame[2] == FRAME_TYPE && frame[3] == FRAME_LENGTH) {
                
                // Calculate and verify CRC (bytes 32-33 are CRC)
                uint16_t received_crc = (uint16_t)frame[32] | ((uint16_t)frame[33] << 8);
                uint16_t calculated_crc = calculate_crc16(frame, 32);
                
                if (received_crc == calculated_crc) {
                    // Parse telemetry data from the frame
                    uint8_t sequence = frame[4];
                    uint8_t device_id = frame[5];
                    
                    // Temperature (FET) - bytes 7-8, -40...+215°C
                    int16_t temp_fet_raw = (int16_t)((uint16_t)frame[7] | ((uint16_t)frame[8] << 8));
                    float temperature_fet = get_temperature_celsius(temp_fet_raw);
                    
                    // Voltage - bytes 9-10, primary voltage (battery)
                    uint16_t voltage_raw = (uint16_t)frame[9] | ((uint16_t)frame[10] << 8);
                    float voltage = voltage_raw / 100.0f; // Convert from centivolt to volt
                    
                    // Current - bytes 11-12, primary current (battery)
                    uint16_t current_raw = (uint16_t)frame[11] | ((uint16_t)frame[12] << 8);
                    float current = current_raw / 10.0f; // Convert from deciampere to ampere
                    
                    // Consumption - bytes 13-14, mAh of primary current
                    uint16_t consumption_raw = (uint16_t)frame[13] | ((uint16_t)frame[14] << 8);
                    float consumption = consumption_raw; // mAh
                    
                    // RPM - bytes 15-16, 0.1erpm
                    uint16_t rpm_raw = (uint16_t)frame[15] | ((uint16_t)frame[16] << 8);
                    float rpm = (rpm_raw / 10.0f) * parameter->rpm_multiplier;
                    
                    // PWM - byte 17, FET-PWM %
                    uint8_t pwm_raw = frame[17];
                    float pwm_percent = pwm_raw; // Already in percentage
                    
                    // Throttle - byte 18, input demand %
                    uint8_t throttle_raw = frame[18];
                    float throttle = throttle_raw; // Already in percentage
                    
                    // BEC voltage - bytes 19-20, in mV
                    uint16_t bec_voltage_raw = (uint16_t)frame[19] | ((uint16_t)frame[20] << 8);
                    float voltage_bec = bec_voltage_raw / 1000.0f; // Convert from mV to V
                    
                    // BEC current - bytes 21-22, in mA
                    uint16_t bec_current_raw = (uint16_t)frame[21] | ((uint16_t)frame[22] << 8);
                    float current_bec = bec_current_raw / 1000.0f; // Convert from mA to A
                    
                    // BEC temperature - bytes 23-24, -40...+215°C (if BEC, -40°C else)
                    int16_t temp_bec_raw = (int16_t)((uint16_t)frame[23] | ((uint16_t)frame[24] << 8));
                    float temperature_bec = get_temperature_celsius(temp_bec_raw);
                    
                    // Update outputs with averaging
                    if (parameter->pwm_out) xTaskNotifyGive(context.pwm_out_task_handle);
                    
                    *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
                    *parameter->consumption = consumption; // Use direct value, not averaged
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
                        "\nOpenYGE (%u) < Seq: %u DevID: %u RPM: %.0f Volt: %.2fV Curr: %.2fA TempFET: %.1f°C "
                        "TempBEC: %.1f°C Cons: %.0fmAh CellV: %.2fV BECVolt: %.2fV BECCurr: %.3fA Thr: %.0f%% PWM: %.0f%%",
                        uxTaskGetStackHighWaterMark(NULL), sequence, device_id, *parameter->rpm, *parameter->voltage, 
                        *parameter->current, *parameter->temperature_fet, *parameter->temperature_bec, *parameter->consumption,
                        *parameter->cell_voltage, *parameter->voltage_bec, *parameter->current_bec, *parameter->throttle, *parameter->pwm_percent);
                    
                    return; // Successfully processed frame
                } else {
                    debug("\nOpenYGE CRC error: expected 0x%04X, got 0x%04X", calculated_crc, received_crc);
                }
            }
        }
    }
}

static uint16_t calculate_crc16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0x0000;
    
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

static float get_temperature_celsius(uint16_t raw_temp) {
    // OpenYGE temperature: -40...+215°C
    // Assuming linear scaling from raw value
    // Need to verify exact scaling from protocol documentation
    int16_t signed_temp = (int16_t)raw_temp;
    
    // If temperature is 0d (-40°C indicator), return 0 for no temperature
    if (signed_temp == 0) return 0;
    
    // Simple conversion - may need adjustment based on actual protocol spec
    float temperature = (float)signed_temp / 10.0f - 40.0f;
    
    if (temperature < -40.0f || temperature > 215.0f) return 0;
    
    return temperature;
}