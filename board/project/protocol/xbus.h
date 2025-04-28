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

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)

typedef enum xbus_sensors_t {
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
} xbus_sensors_t;

typedef enum xbus_airspeed_enum_t { XBUS_AIRSPEED_AIRSPEED } xbus_airspeed_enum_t;

typedef enum xbus_altitude_enum_t { XBUS_ALTITUDE } xbus_altitude_enum_t;

typedef enum xbus_gps_loc_enum_t {
    XBUS_GPS_LOC_ALTITUDE,
    XBUS_GPS_LOC_LATITUDE,
    XBUS_GPS_LOC_LONGITUDE,
    XBUS_GPS_LOC_COURSE,
    XBUS_GPS_LOC_HDOP
} xbus_gps_loc_enum_t;

typedef enum xbus_gps_stat_enum_t {
    XBUS_GPS_STAT_SPEED,
    XBUS_GPS_STAT_TIME,
    XBUS_GPS_STAT_SATS,
    XBUS_GPS_STAT_ALTITUDE
} xbus_gps_stat_enum_t;

typedef enum xbus_energy_enum_t {
    XBUS_ENERGY_CURRENT1,
    XBUS_ENERGY_CONSUMPTION1,
    XBUS_ENERGY_VOLTAGE1,
    XBUS_ENERGY_CURRENT2,
    XBUS_ENERGY_CONSUMPTION2,
    XBUS_ENERGY_VOLTAGE2
} xbus_energy_enum_t;

typedef enum xbus_esc_enum_t {
    XBUS_ESC_RPM,
    XBUS_ESC_VOLTAGE,
    XBUS_ESC_CURRENT,
    XBUS_ESC_TEMPERATURE_FET,
    XBUS_ESC_TEMPERATURE_BEC,
    XBUS_ESC_VOLTAGE_BEC,
    XBUS_ESC_CURRENT_BEC
} xbus_esc_enum_t;

typedef enum xbus_battery_enum_t {
    XBUS_BATTERY_CURRENT1,
    XBUS_BATTERY_CONSUMPTION1,
    XBUS_BATTERY_TEMP1,
    XBUS_BATTERY_CURRENT2,
    XBUS_BATTERY_CONSUMPTION2,
    XBUS_BATTERY_TEMP2
} xbus_battery_enum_t;

typedef enum xbus_vario_enum_t { XBUS_VARIO_ALTITUDE } xbus_vario_enum_t;

typedef enum xbus_rpm_volt_temp_enum_t {
    XBUS_RPMVOLTTEMP_MS,
    XBUS_RPMVOLTTEMP_VOLT,
    XBUS_RPMVOLTTEMP_TEMP
} xbus_rpm_volt_temp_enum_t;

typedef enum xbus_fuel_flow_enum_t { XBUS_FUEL_FLOW_CONSUMED, XBUS_FUEL_FLOW_RATE } xbus_fuel_flow_enum_t;

typedef enum xbus_stru_tele_digital_air_enum_t { XBUS_FUEL_PRESSURE } xbus_stru_tele_digital_air_enum_t;

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
    uint8_t id;                // Source device = 0x22
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
    uint8_t id;         // Source device = 0x36
    uint8_t sID;        // Secondary ID
    uint16_t digital;   // Digital inputs (bit per input)
    uint16_t pressure;  // Tank pressure, 0.1PSI (0-6553.4PSI)
} xbus_stru_tele_digital_air_t;

typedef struct xbus_sensor_formatted_t {
    xbus_airspeed_t *airspeed;
    xbus_altitude_t *altitude;
    xbus_gps_loc_t *gps_loc;
    xbus_gps_stat_t *gps_stat;
    xbus_esc_t *esc;
    xbus_battery_t *battery;
    xbus_vario_t *vario;
    xbus_rpm_volt_temp_t *rpm_volt_temp;
    xbus_energy_t *energy;
    xbus_fuel_flow_t *fuel_flow;
    xbus_stru_tele_digital_air_t *stru_tele_digital_air;
} xbus_sensor_formatted_t;

typedef struct xbus_sensor_t {
    bool is_enabled[11];
    float *airspeed[1];
    float *altimeter[1];
    float *gps_loc[5];
    float *gps_stat[4];
    float *esc[7];
    float *battery[6];
    float *vario[1];
    float *rpm_volt_temp[3];
    float *energy[6];
    float *fuel_flow[2];
    float *stru_tele_digital_air[1];
} xbus_sensor_t;

extern context_t context;

void xbus_task(void *parameters);
void xbus_format_sensor(uint8_t address);

#ifdef SIM_RX
void xbus_i2c_handler(uint8_t address);
#endif

#endif