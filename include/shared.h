#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>

#ifdef __cplusplus

typedef enum rx_protocol_t : uint8_t {
    RX_SMARTPORT,
    RX_FRSKY_D,
    RX_XBUS,
    RX_SRXL,
    RX_IBUS,
    RX_SBUS,
    RX_MULTIPLEX,
    RX_JETIEX,
    RX_HITEC,
    RX_SRXL2,
    SERIAL_MONITOR,
    RX_CRSF,
    RX_HOTT,
    RX_SANWA,
    RX_JR_PROPO,
    RX_FPORT,
    RX_FBUS,
    RX_GHST
} rx_protocol_t;

typedef enum esc_protocol_t : uint8_t {
    ESC_NONE,
    ESC_HW3,
    ESC_HW4,
    ESC_PWM,
    ESC_CASTLE,
    ESC_KONTRONIK,
    ESC_APD_F,
    ESC_APD_HV,
    ESC_HW5,
    ESC_SMART,
    ESC_OMP_M4,
    ESC_ZTW,
    ESC_OPENYGE
} esc_protocol_t;

typedef enum i2c_module_t : uint8_t { I2C_NONE, I2C_BMP280, I2C_MS5611, I2C_BMP180 } i2c_module_t;

typedef enum analog_current_type_t : uint8_t { CURRENT_TYPE_HALL, CURRENT_TYPE_SHUNT } analog_current_type_t;

typedef enum serial_monitor_format_t : uint8_t { FORMAT_HEX, FORMAT_STRING } serial_monitor_format_t;

typedef enum gps_protocol_t : uint8_t { UBLOX, NMEA } gps_protocol_t;

#else

typedef enum rx_protocol_t {
    RX_SMARTPORT,
    RX_FRSKY_D,
    RX_XBUS,
    RX_SRXL,
    RX_IBUS,
    RX_SBUS,
    RX_MULTIPLEX,
    RX_JETIEX,
    RX_HITEC,
    RX_SRXL2,
    SERIAL_MONITOR,
    RX_CRSF,
    RX_HOTT,
    RX_SANWA,
    RX_JR_PROPO,
    RX_FPORT,
    RX_FBUS,
    RX_GHST
} rx_protocol_t;

typedef enum esc_protocol_t {
    ESC_NONE,
    ESC_HW3,
    ESC_HW4,
    ESC_PWM,
    ESC_CASTLE,
    ESC_KONTRONIK,
    ESC_APD_F,
    ESC_APD_HV,
    ESC_HW5,
    ESC_SMART,
    ESC_OMP_M4,
    ESC_ZTW,
    ESC_OPENYGE
} esc_protocol_t;

typedef enum i2c_module_t { I2C_NONE, I2C_BMP280, I2C_MS5611, I2C_BMP180 } i2c_module_t;

typedef enum analog_current_type_t { CURRENT_TYPE_HALL, CURRENT_TYPE_SHUNT } analog_current_type_t;

typedef enum serial_monitor_format_t { FORMAT_HEX, FORMAT_STRING } serial_monitor_format_t;

typedef enum gps_protocol_t { UBLOX, NMEA } gps_protocol_t;

#endif

typedef struct config_t {                            // smartport data_id
    uint16_t version;                                // 0x5101
    enum rx_protocol_t rx_protocol;                  // 0x5102
    enum esc_protocol_t esc_protocol;                // 0x5103
    bool enable_gps;                                 // 0x5104
    uint32_t gps_baudrate;                           // 0x5105
    bool enable_analog_voltage;                      // 0x5106
    bool enable_analog_current;                      // 0x5107
    bool enable_analog_ntc;                          // 0x5108
    bool enable_analog_airspeed;                     // 0x5109
    enum i2c_module_t i2c_module;                    // 0x510A
    uint8_t spare0;
    float alpha_rpm;                                 // 0x510C
    float alpha_voltage;                             // 0x510D
    float alpha_current;                             // 0x510E
    float alpha_temperature;                         // 0x510F
    float alpha_vario;                               // 0x5110
    float alpha_airspeed;                            // 0x5111
    uint16_t refresh_rate_rpm;                       // 0x5112
    uint16_t refresh_rate_voltage;                   // 0x5113
    uint16_t refresh_rate_current;                   // 0x5114
    uint16_t refresh_rate_temperature;               // 0x5115
    uint16_t refresh_rate_gps;                       // 0x5116
    uint16_t refresh_rate_consumption;               // 0x5117
    uint16_t refresh_rate_vario;                     // 0x5118
    uint16_t refresh_rate_airspeed;                  // 0x5119
    uint16_t refresh_rate_default;                   // 0x511A
    float analog_voltage_multiplier;                 // 0x511B
    enum analog_current_type_t analog_current_type;  // 0x511C
    uint16_t gpio_interval;                          // 0x511D
    float analog_current_quiescent_voltage;          // 0x511E
    float analog_current_multiplier;                 // 0x511F
    float analog_current_offset;                     // 0x5120
    bool analog_current_autoffset;                   // 0x5121
    uint8_t pairOfPoles;                             // 0x5122
    uint8_t mainTeeth;                               // 0x5123
    uint8_t pinionTeeth;                             // 0x5124
    float rpm_multiplier;                            // 0x5125
    uint8_t bmp280_filter;                           // 0x5126
    bool enable_pwm_out;                             // 0x5127
    uint8_t smartport_sensor_id;                     // 0x5128
    uint16_t smartport_data_id;                      // 0x5129
    bool vario_auto_offset;                          // 0x512A
    bool xbus_clock_stretch;                         // 0x512B
    bool jeti_gps_speed_units_kmh;                   // 0x512C
    bool enable_esc_hw4_init_delay;                  // 0x512D
    uint16_t esc_hw4_init_delay_duration;            // 0x512E
    uint8_t esc_hw4_current_thresold;                // 0x512F
    uint16_t esc_hw4_current_max;                    // 0x5130
    float esc_hw4_voltage_multiplier;                // 0x5131
    float esc_hw4_current_multiplier;                // 0x5132
    bool ibus_alternative_coordinates;               // 0x5133
    uint8_t debug;                                   // 0x5134
    bool esc_hw4_is_manual_offset;                   // 0x5135
    uint8_t analog_rate;                             // 0x5136
    bool xbus_use_alternative_volt_temp;             // 0x5137
    uint8_t gpio_mask;                               // 0x5138
    float esc_hw4_offset;                            // 0x5139
    uint32_t serial_monitor_baudrate;                // 0x513A
    uint8_t serial_monitor_stop_bits;                // 0x513B
    uint8_t serial_monitor_parity;                   // 0x513C
    uint16_t serial_monitor_timeout_ms;              // 0x513D
    bool serial_monitor_inverted;                    // 0x513E
    bool esc_hw4_auto_detect;                        // 0x514B
    int16_t airspeed_vcc;                            // 0x5140
    float fuel_flow_ml_per_pulse;                    // 0x5141
    bool enable_fuel_flow;                           // 0x5142
    uint16_t xgzp68xxd_k;                            // 0x5143
    uint8_t enable_fuel_pressure;                    // 0x5144
    bool smart_esc_calc_consumption;                 // 0x5145
    uint8_t serial_monitor_gpio;                     // 0x5146
    uint8_t gps_rate;                                // 0x5147
    serial_monitor_format_t serial_monitor_format;   // 0x5148
    uint8_t gps_protocol;                            // 0x5149
    bool sbus_battery_slot;                          // 0x514A
    bool fport_inverted;
    bool fbus_inverted;
    int16_t airspeed_offset;                         // 0x513F
    uint8_t mpu6050_acc_scale;                       // 0x514C
    uint8_t mpu6050_gyro_scale;                      // 0x514D
    uint8_t mpu6050_gyro_weighting;                  // 0x514E
    bool enable_gyro;                                // 0x514F
    uint8_t spare1;
    uint8_t mpu6050_filter;                          // 0x5151
    uint32_t spare7;
    uint32_t spare8;
    uint32_t spare9;
    uint32_t spare10;
    uint32_t spare11;
    uint32_t spare12;
    uint32_t spare13;
    uint32_t spare14;
    uint32_t spare15;
    uint32_t spare16;
    uint32_t spare17;
    uint32_t spare18;
    uint32_t spare19;
    uint32_t spare20;
} config_t;

#endif