#ifndef XBUS_H
#define XBUS_H

#include "common.h"

#define XBUS_AIRSPEED_ID 0x11
#define XBUS_ALTIMETER_ID 0x12
#define XBUS_GPS_LOC_ID 0x16
#define XBUS_GPS_STAT_ID 0x17
#define XBUS_ENERGY_ID 0x18
#define XBUS_ESC_ID 0x20
#define XBUS_BATTERY_ID 0x34
#define XBUS_VARIO_ID 0X40
#define XBUS_RPMVOLTTEMP_ID 0x7E
#define XBUS_FUEL_FLOW_ID 0x22
#define XBUS_STRU_TELE_DIGITAL_AIR_ID 0x36

#define XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT 0
#define XBUS_GPS_INFO_FLAGS_IS_EAST_BIT 1
#define XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT 2
#define XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT 7

typedef enum xbus_sensors_enabled_t {
    XBUS_AIRSPEED,
    XBUS_ALTIMETER,
    XBUS_GPS_LOC,
    XBUS_GPS_STAT,
    XBUS_ESC,
    XBUS_BATTERY,
    XBUS_VARIO,
    XBUS_RPMVOLTTEMP,
    XBUS_ENERGY,
    XBUS_FUEL_FLOW,
    XBUS_STRU_TELE_DIGITAL_AIR
} xbus_sensors_enabled_t;

typedef struct xbus_airspeed_t {
    uint8_t identifier;     // Source device 0x11
    uint8_t s_id;           // Secondary ID
    uint16_t airspeed;      // 1 km/h increments
    uint16_t max_airspeed;  // 1 km/h increments
} xbus_airspeed_t;

typedef struct xbus_altitude_t {
    uint8_t identifier;    // Source device 0x12
    uint8_t s_id;          // Secondary ID
    int16_t altitude;      // .1m increments
    int16_t max_altitude;  // .1m increments
} xbus_altitude_t;

typedef struct xbus_gps_loc_t {
    uint8_t identifier;     // Source device 0x16
    uint8_t s_id;           // Secondary ID
    uint16_t altitude_low;  // BCD, meters, format 3.1 (Low bits of alt)
    uint32_t latitude;      // BCD, format 4.4, // Degrees * 100 + minutes, < 100 degrees 1234.1234
    uint32_t longitude;     // BCD, format 4.4, // Degrees * 100 + minutes, flag --> > 99deg
    uint16_t course;        // BCD, 3.1
    uint8_t hdop;           // BCD, format 1.1
    uint8_t gps_flags;      // see definitions below
} xbus_gps_loc_t;

typedef struct xbus_gps_stat_t {
    uint8_t identifier;     // Source device 0x17
    uint8_t s_id;           // Secondary ID
    uint16_t speed;         // BCD, knots, format 3.1
    uint32_t utc;           // BCD, format HH:MM:SS.SS, format 6.2
    uint8_t num_sats;       // BCD, 0-99
    uint8_t altitude_high;  // BCD, meters, format 2.0 (High bits alt)
} xbus_gps_stat_t;

typedef struct xbus_energy_t {
    uint8_t identifier;     // Source device 0x18
    uint8_t s_id;           // Secondary ID
    int16_t current_a;      // Instantaneous current, 0.01A (0-327.7A)
    int16_t charge_used_a;  // Integrated mAh used, 0.1mAh (0-3276.6mAh)
    uint16_t volts_a;       // Voltage, 0.01VC (0-16.00V)
    int16_t current_b;      // Instantaneous current, 0.01A (0-327.7A)
    int16_t charge_used_b;  // Integrated mAh used, 0.1mAh (0-3276.6mAh)
    uint16_t volts_b;       // Voltage, 0.01VC (0-16.00V)
} xbus_energy_t;

typedef struct xbus_esc_t {
    uint8_t identifier;      // Source device 0x20
    uint8_t s_id;            // Secondary ID
    uint16_t rpm;            // RPM, 10RPM (0-655340 RPM).0xFFFF -->
    uint16_t volts_input;    // Volts, 0.01v (0-655.34V).0xFFFF -->
    uint16_t temp_fet;       // Temperature, 0.1C (0-999.8C)0xFFFF -->
    uint16_t current_motor;  // Current, 10mA (0-655.34A).0xFFFF -->
    uint16_t temp_bec;       // Temperature, 0.1C (0-999.8C)0x7FFF -->
    uint8_t current_bec;     // BEC Current, 100mA (0-25.4A). 0xFF ---->
    uint8_t voltage_bec;     // BEC Volts, 0.05V (0-12.70V). 0xFF ---->
    uint8_t throttle;        // 0.5% (0-127%). 0xFF ---->
    uint8_t power_out;       // Power Output, 0.5% (0-127%). 0xFF ---->
} xbus_esc_t;

typedef struct xbus_battery_t {
    uint8_t identifier;     // Source device 0x34
    uint8_t s_id;           // Secondary ID
    int16_t current_a;      // Instantaneous current, 0.1A (0-3276.8A)
    int16_t charge_used_a;  // Integrated mAh used, 1mAh (0-32.766Ah)
    uint16_t temp_a;        // Temperature, 0.1C (0-150.0C, // 0x7FFF indicates not populated)
    int16_t current_b;      // Instantaneous current, 0.1A (0-6553.4A)
    int16_t charge_used_b;  // Integrated mAh used, 1mAh (0-65.534Ah)
    uint16_t temp_b;        // Temperature, 0.1C (0-150.0C,// 0x7FFF indicates not populated)
} xbus_battery_t;

typedef struct xbus_vario_t {
    uint8_t identifier;    // Source device 0x40
    uint8_t s_id;          // Secondary ID
    int16_t altitude;      // .1m increments
    int16_t delta_0250ms,  // delta last 250ms, 0.1m/s increments
        delta_0500ms,      // delta last 500ms, 0.1m/s increments
        delta_1000ms,      // delta last 1.0 seconds
        delta_1500ms,      // delta last 1.5 seconds
        delta_2000ms,      // delta last 2.0 seconds
        delta_3000ms;      // delta last 3.0 seconds
} xbus_vario_t;

typedef struct xbus_rpm_volt_temp_t {
    uint8_t identifier;  // Source device 0x7E
    uint8_t s_id;
    uint16_t microseconds;  // microseconds between pulse leading edges
    uint16_t volts;         // 0.01V increments
    int16_t temperature;    // degrees F
} xbus_rpm_volt_temp_t;

typedef struct xbus_fuel_flow_t {
    uint8_t identifier;        // Source device = 0x22
    uint8_t s_id;              // Secondary ID
    uint16_t fuel_consumed_A;  // Integrated fuel consumption, 0.1mL
    uint16_t flow_rate_A;      // Instantaneous consumption, 0.01mL/min
    uint16_t temp_A;           // Temperature, 0.1C (0-655.34C)
    uint16_t fuel_consumed_B;  // Integrated fuel consumption, 0.1mL
    uint16_t flow_rate_B;      // Instantaneous consumption, 0.01mL/min
    uint16_t temp_B;           // Temperature, 0.1C (0-655.34C)
    uint16_t spare;            // Not used
} xbus_fuel_flow_t;

typedef struct xbus_stru_tele_digital_air_t {
    uint8_t identifier;  // Source device = 0x36
    uint8_t sID;         // Secondary ID
    uint16_t digital;    // Digital inputs (bit per input)
    uint16_t pressure;   // Tank pressure, 0.1PSI (0-6553.4PSI)
} xbus_stru_tele_digital_air_t;

typedef struct xbus_sensor_airspeed_t {
    float *speed;
} xbus_sensor_airspeed_t;

typedef struct xbus_sensor_altimeter_t {
    float *altitude;
} xbus_sensor_altimeter_t;

typedef struct xbus_sensor_gps_loc_t {
    float *altitude;
    double *latitude;
    double *longitude;
    float *course;
    float *hdop;
} xbus_sensor_gps_loc_t;

typedef struct xbus_sensor_gps_stat_t {
    float *speed;
    float *time;
    float *sats;
    float *altitude;
} xbus_sensor_gps_stat_t;

typedef struct xbus_sensor_esc_t {
    float *rpm;
    float *voltage;
    float *current;
    float *temp_fet;
    float *temp_bec;
    float *voltage_bec;
    float *current_bec;
} xbus_sensor_esc_t;

typedef struct xbus_sensor_battery_t {
    float *current1;
    float *consumption1;
    float *temp1;
    float *current2;
    float *consumption2;
    float *temp2;
} xbus_sensor_battery_t;

typedef struct xbus_sensor_vario_t {
    float *altitude;
    float delta_0250ms;
    float delta_0500ms;
    float delta_1000ms;
    float delta_1500ms;
    float delta_2000ms;
    float delta_3000ms;
} xbus_sensor_vario_t;

typedef struct xbus_sensor_rpmvolttemp_t {
    float *rpm;
    float *volt;
    float *temp;
} xbus_sensor_rpmvolttemp_t;

typedef struct xbus_sensor_energy_t {
    float *current1;
    float *consumption1;
    float *voltage1;
    float *current2;
    float *consumption2;
    float *voltage2;
} xbus_sensor_energy_t;

typedef struct xbus_sensor_fuelflow_t {
    float *consumed;
    float *flow_rate;
} xbus_sensor_fuelflow_t;

typedef struct xbus_sensor_stru_tele_digital_air_t {
    float *fuel_pressure;
} xbus_sensor_stru_tele_digital_air_t;

typedef struct xbus_sensors_t {
    bool is_enabled[11];
    xbus_sensor_airspeed_t airspeed;
    xbus_sensor_altimeter_t altimeter;
    xbus_sensor_gps_loc_t gps_loc;
    xbus_sensor_gps_stat_t gps_stat;
    xbus_sensor_esc_t esc;
    xbus_sensor_battery_t battery;
    xbus_sensor_vario_t vario;
    xbus_sensor_rpmvolttemp_t rpm_volt_temp;
    xbus_sensor_energy_t energy;
    xbus_sensor_fuelflow_t fuel_flow;
    xbus_sensor_stru_tele_digital_air_t stru_tele_digital_air;
} xbus_sensors_t;

extern context_t context;

void xbus_task(void *parameters);
void xbus_format_sensor(uint8_t address, uint8_t *buffer);
void xbus_set_config(void);

#ifdef SIM_RX
void xbus_i2c_handler(uint8_t address);
#endif

#endif