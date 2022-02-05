#ifndef FORMATDATA_H
#define FORMATDATA_H

// FrSky Smartport DATA IDs (2 bytes)
#define ALT_FIRST_ID 0x0100 // 100 m
#define ALT_LAST_ID 0x010f
#define VARIO_FIRST_ID 0x0110 // 100 m/s
#define VARIO_LAST_ID 0x011f
#define CURR_FIRST_ID 0x0200 // 10 A
#define CURR_LAST_ID 0x020f
#define VFAS_FIRST_ID 0x0210 // 100 v
#define VFAS_LAST_ID 0x021f
#define CELLS_FIRST_ID 0x0300 //
#define CELLS_LAST_ID 0x030f
#define T1_FIRST_ID 0x0400 // 1 C
#define T1_LAST_ID 0x040f
#define T2_FIRST_ID 0x0410 // 1 C
#define T2_LAST_ID 0x041f
#define RPM_FIRST_ID 0x0500 // 1 rpm
#define RPM_LAST_ID 0x050f
#define FUEL_FIRST_ID 0x0600 // 1 %
#define FUEL_LAST_ID 0x060f
#define ACCX_FIRST_ID 0x0700 // 100 g
#define ACCX_LAST_ID 0x070f
#define ACCY_FIRST_ID 0x0710 // 100 g
#define ACCY_LAST_ID 0x071f
#define ACCZ_FIRST_ID 0x0720 // 100 g
#define ACCZ_LAST_ID 0x072f
#define GPS_LONG_LATI_FIRST_ID 0x0800 // bit32(1<<31)=1=LON:=0=LAT, bit31(1<<30)=1=-:=0=+, escaler: 10000
#define GPS_LONG_LATI_LAST_ID 0x080f
#define GPS_ALT_FIRST_ID 0x0820 // 100 m
#define GPS_ALT_LAST_ID 0x082f
#define GPS_SPEED_FIRST_ID 0x0830 // 1000 kts
#define GPS_SPEED_LAST_ID 0x083f
#define GPS_COURS_FIRST_ID 0x0840 // 100 º
#define GPS_COURS_LAST_ID 0x084f
#define GPS_TIME_DATE_FIRST_ID 0x0850 // Date: Y M D 0xFF or Time: H M S 0x00
#define GPS_TIME_DATE_LAST_ID 0x085f
#define A3_FIRST_ID 0x0900 // 100 v
#define A3_LAST_ID 0x090f
#define A4_FIRST_ID 0x0910 // 100 v
#define A4_LAST_ID 0x091f
#define AIR_SPEED_FIRST_ID 0x0a00 // 10 kts
#define AIR_SPEED_LAST_ID 0x0a0f
#define RBOX_BATT1_FIRST_ID 0x0b00 // 1000 v, 100 A
#define RBOX_BATT1_LAST_ID 0x0b0f
#define RBOX_BATT2_FIRST_ID 0x0b10 // 1000 v, 100 A
#define RBOX_BATT2_LAST_ID 0x0b1f
#define RBOX_STATE_FIRST_ID 0x0b20 // 1
#define RBOX_STATE_LAST_ID 0x0b2f
#define RBOX_CNSP_FIRST_ID 0x0b30 // 1 mAh (1), 1mAh (2)
#define RBOX_CNSP_LAST_ID 0x0b3f
#define SD1_FIRST_ID 0x0b40
#define SD1_LAST_ID 0x0b4f
#define ESC_POWER_FIRST_ID 0x0b50 // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define ESC_POWER_LAST_ID 0x0b5f
#define ESC_RPM_CONS_FIRST_ID 0x0b60 // bytes 1,2: 0.01 rpm,  bytes 3,4: 1 mah
#define ESC_RPM_CONS_LAST_ID 0x0b6f
#define ESC_TEMPERATURE_FIRST_ID 0x0b70 // 1 C
#define ESC_TEMPERATURE_LAST_ID 0x0b7f
#define X8R_FIRST_ID 0x0c20
#define X8R_LAST_ID 0x0c2f
#define S6R_FIRST_ID 0x0c30
#define S6R_LAST_ID 0x0c3f
#define GASSUIT_TEMP1_FIRST_ID 0x0d00 // 1 C
#define GASSUIT_TEMP1_LAST_ID 0x0d0f
#define GASSUIT_TEMP2_FIRST_ID 0x0d10 // 1 C
#define GASSUIT_TEMP2_LAST_ID 0x0d1f
#define GASSUIT_SPEED_FIRST_ID 0x0d20 // 1 rpm
#define GASSUIT_SPEED_LAST_ID 0x0d2f
#define GASSUIT_RES_VOL_FIRST_ID 0x0d30 // 1 ml
#define GASSUIT_RES_VOL_LAST_ID 0x0d3f
#define GASSUIT_RES_PERC_FIRST_ID 0x0d40 // 1 %
#define GASSUIT_RES_PERC_LAST_ID 0x0d4f
#define GASSUIT_FLOW_FIRST_ID 0x0d50 // 1 ml
#define GASSUIT_FLOW_LAST_ID 0x0d5f
#define GASSUIT_MAX_FLOW_FIRST_ID 0x0d60 // 1 ml
#define GASSUIT_MAX_FLOW_LAST_ID 0x0d6f
#define GASSUIT_AVG_FLOW_FIRST_ID 0x0d70 // 1 ml
#define GASSUIT_AVG_FLOW_LAST_ID 0x0d7f
#define SBEC_POWER_FIRST_ID 0x0e50 // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define SBEC_POWER_LAST_ID 0x0e5f
#define DIY_FIRST_ID 0x5100
#define DIY_LAST_ID 0x52ff
#define DIY_STREAM_FIRST_ID 0x5000
#define DIY_STREAM_LAST_ID 0x50ff
#define FACT_TEST_ID 0xf000
#define RSSI_ID 0xf101
#define A1_ID 0xf102 // 10 v
#define A2_ID 0xf103 // 10 v
#define SP2UART_A_ID 0xfd00
#define SP2UART_B_ID 0xfd01
#define RXBT_ID 0xf104 // 10 v
#define RAS_ID 0xf105
#define XJT_VERSION_ID 0xf106
#define FUEL_QTY_FIRST_ID 0x0a10 // 100 ml
#define FUEL_QTY_LAST_ID 0x0a1f

// FrSky D DATA IDs (1 byte)
#define GPS_ALT_BP_ID 0x01
#define TEMP1_ID 0x02
#define RPM_ID 0x03
#define FUEL_ID 0x04
#define TEMP2_ID 0x05
#define CELL_VOLT_ID 0x06
#define GPS_ALT_AP_ID 0x09
#define BARO_ALT_BP_ID 0x10
#define GPS_SPEED_BP_ID 0x11
#define GPS_LONG_BP_ID 0x12
#define GPS_LAT_BP_ID 0x13
#define GPS_COURS_BP_ID 0x14
#define GPS_DAY_MONTH_ID 0x15
#define GPS_YEAR_ID 0x16
#define GPS_HOUR_MIN_ID 0x17
#define GPS_SEC_ID 0x18
#define GPS_SPEED_AP_ID 0x19
#define GPS_LONG_AP_ID 0x1A
#define GPS_LAT_AP_ID 0x1B
#define GPS_COURS_AP_ID 0x1C
#define BARO_ALT_AP_ID 0x21
#define GPS_LONG_EW_ID 0x22
#define GPS_LAT_NS_ID 0x23
#define ACCEL_X_ID 0x24
#define ACCEL_Y_ID 0x25
#define ACCEL_Z_ID 0x26
#define CURRENT_ID 0x28
#define VARIO_ID 0x30
#define VFAS_ID 0x39
#define VOLTS_BP_ID 0x3A
#define VOLTS_AP_ID 0x3B
#define FRSKY_LAST_ID 0x3F
#define D_RSSI_ID 0xF0
#define D_A1_ID 0xF1
#define D_A2_ID 0xF2

// Flysky IBUS Data Id
#define AFHDS2A_ID_VOLTAGE 0x00        // Internal Voltage
#define AFHDS2A_ID_TEMPERATURE 0x01    // Temperature
#define AFHDS2A_ID_MOT 0x02            // RPM
#define AFHDS2A_ID_EXTV 0x03           // External Voltage
#define AFHDS2A_ID_CELL_VOLTAGE 0x04   // Avg Cell voltage
#define AFHDS2A_ID_BAT_CURR 0x05       // battery current A * 100
#define AFHDS2A_ID_FUEL 0x06           // remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define AFHDS2A_ID_RPM 0x07            // throttle value / battery capacity
#define AFHDS2A_ID_CMP_HEAD 0x08       // Heading  0..360 deg 0north 2bytes
#define AFHDS2A_ID_CLIMB_RATE 0x09     // 2 bytes m/s *100 signed
#define AFHDS2A_ID_COG 0x0A            // 2 bytes  Course over ground(NOT heading but direction of movement) in degrees * 100 0.0..359.99 degrees. unknown max uint
#define AFHDS2A_ID_GPS_STATUS 0x0B     // 2 bytes
#define AFHDS2A_ID_ACC_X 0x0C          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ACC_Y 0x0D          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ACC_Z 0x0E          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ROLL 0x0F           // 2 bytes deg *100 signed
#define AFHDS2A_ID_PITCH 0x10          // 2 bytes deg *100 signed
#define AFHDS2A_ID_YAW 0x11            // 2 bytes deg *100 signed
#define AFHDS2A_ID_VERTICAL_SPEED 0x12 // 2 bytes m/s *100 signed
#define AFHDS2A_ID_GROUND_SPEED 0x13   // 2 bytes m/s *100 different unit than build-in sensor
#define AFHDS2A_ID_GPS_DIST 0x14       // 2 bytes distance from home m unsigned
#define AFHDS2A_ID_ARMED 0x15          // 2 bytes
#define AFHDS2A_ID_FLIGHT_MODE 0x16    // 2 bytes
#define AFHDS2A_ID_PRES 0x41           // Pressure
#define AFHDS2A_ID_ODO1 0x7C           // Odometer1
#define AFHDS2A_ID_ODO2 0x7D           // Odometer2
#define AFHDS2A_ID_SPE 0x7E            // Speed 2 bytes km/h
#define AFHDS2A_ID_TX_V 0x7F           // TX Voltage

#define AFHDS2A_ID_GPS_LAT 0x80 // 4bytes signed WGS84 in degrees * 1E7
#define AFHDS2A_ID_GPS_LON 0x81 // 4bytes signed WGS84 in degrees * 1E7
#define AFHDS2A_ID_GPS_ALT 0x82 // 4bytes signed!!! GPS alt m*100
#define AFHDS2A_ID_ALT 0x83     // 4bytes signed!!! Alt m*100
#define AFHDS2A_ID_S84 0x84
#define AFHDS2A_ID_S85 0x85
#define AFHDS2A_ID_S86 0x86
#define AFHDS2A_ID_S87 0x87
#define AFHDS2A_ID_S88 0x88
#define AFHDS2A_ID_S89 0x89
#define AFHDS2A_ID_S8a 0x8A
#define AFHDS2A_ID_RX_SIG_AFHDS3 0xF7 // SIG
#define AFHDS2A_ID_RX_SNR_AFHDS3 0xF8 // SNR
#define AFHDS2A_ID_ALT_FLYSKY 0xF9    // Altitude 2 bytes signed in m - used in FlySky native TX
#define AFHDS2A_ID_RX_SNR 0xFA        // SNR
#define AFHDS2A_ID_RX_NOISE 0xFB      // Noise
#define AFHDS2A_ID_RX_RSSI 0xFC       // RSSI
#define AFHDS2A_ID_RX_ERR_RATE 0xFE   // Error rate
#define AFHDS2A_ID_END 0xFF

// AC type telemetry with multiple values in one packet
#define AFHDS2A_ID_GPS_FULL 0xFD
#define AFHDS2A_ID_VOLT_FULL 0xF0
#define AFHDS2A_ID_ACC_FULL 0xEF
#define AFHDS2A_ID_TX_RSSI 0x200 // Pseudo id outside 1 byte range of FlySky sensors

// FASST Sbus
#define FASST_NULL 0
#define FASST_TEMP 1
#define FASST_VOLT_V1 2
#define FASST_VOLT_V2 3
#define FASST_RPM 4
#define FASST_POWER_CURR 5
#define FASST_POWER_VOLT 6
#define FASST_POWER_CONS 7
#define FASST_VARIO_ALT 8 // F1672
#define FASST_VARIO_SPEED 9
#define FASST_GPS_SPEED 10  // F1675
#define FASST_GPS_ALTITUDE 11
#define FASST_GPS_TIME 12
#define FASST_GPS_VARIO_SPEED 13
#define FASST_GPS_LATITUDE1 14
#define FASST_GPS_LATITUDE2 15
#define FASST_GPS_LONGITUDE1 16
#define FASST_GPS_LONGITUDE2 17

#define FASST_NEGATIVE_BIT 15
#define FASST_SOUTH_WEST_BIT 4

// MULTIPLEX FHSS
#define FHSS_VOLTAGE 1
#define FHSS_CURRENT 2
#define FHSS_VARIO 3
#define FHSS_SPEED 4
#define FHSS_RPM 5
#define FHSS_TEMP 6
#define FHSS_COURSE 7
#define FHSS_ALTITUDE 8
#define FHSS_LEVEL 9
#define FHSS_RSSI 10
#define FHSS_CONSUMPTION 11
#define FHSS_FLUID 12
#define FHSS_DISTANCE 13

#define TYPE_LAT 0
#define TYPE_LON 1
#define TYPE_DATE 0
#define TYPE_TIME 1

#include <Arduino.h>

class FormatData
{
protected:
    uint32_t formatData(uint16_t dataId, float valueL, float valueM);
    uint32_t formatData(uint16_t dataId, float valueL);
    uint32_t formatLatLon(uint8_t type, float value);
    uint32_t formatDateTime(uint8_t type, uint32_t value);
    uint32_t formatCell(uint8_t cellIndex, float value1, float value2);
    uint16_t formatData(uint8_t dataId, float value);
    int32_t formatIbus(uint8_t dataId, float value);
    uint16_t formatSbus(uint8_t dataId, float value);
    int16_t formatMultiplex(uint8_t dataId, float value);
};

#endif