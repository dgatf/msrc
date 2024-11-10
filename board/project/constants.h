#ifndef CONSTANTS_H
#define CONSTANTS_H

/* Version */
#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define VERSION_PATCH 0

/* Pins */
#define UART0_TX_GPIO 0       // receiver rx
#define UART0_RX_GPIO 1       // receiver tx
#define I2C1_SDA_GPIO 2       // receiver sda (with pull up)
#define I2C1_SCL_GPIO 3       // receiver scl (with pull up)
#define UART1_TX_GPIO 4       // not used
#define UART1_RX_GPIO 5       // esc tx
#define PWM_CAPTURE_GPIO 4    // pwm (rpm)
#define CASTLE_RX_GPIO 4      // receiver throttle signal
#define CASTLE_ESC_GPIO 5     // esc throttle signal
#define UART_RX_PIO_GPIO 6    // gps tx
#define CLOCK_STRETCH_GPIO 7  // npn switch
#define I2C0_SDA_GPIO 8       // i2c module (vario)
#define I2C0_SCL_GPIO 9       // i2c module (vario)
#define PWM_OUT_GPIO 10       // pwm out
#define ADC0_GPIO 26          // Voltage
#define ADC1_GPIO 27          // Current
#define ADC2_GPIO 28          // NTC
#define ADC3_GPIO 29          // Airspeed

/* UARTS */
#define UART_RECEIVER uart0
#define UART_RECEIVER_RX UART0_RX_GPIO
#define UART_RECEIVER_TX UART0_TX_GPIO
#define UART_ESC uart1
#define UART_ESC_RX UART1_RX_GPIO
#define UART_ESC_TX UART1_TX_GPIO

// set receiver to uart1 when debugging with probe
// #define UART_RECEIVER uart1
// #define UART_RECEIVER_RX UART1_RX_GPIO
// #define UART_RECEIVER_TX UART1_TX_GPIO

/* ADC */
#define ANALOG_SENSOR_INTERVAL_MS 500
#define BOARD_VCC 3.3
#define ADC_RESOLUTION 4096

/* Stack */
#define STACK_EXTRA 100

#define STACK_RX_IBUS (200 + STACK_EXTRA)
#define STACK_RX_FRSKY_D (654 + STACK_EXTRA)
#define STACK_RX_MULTIPLEX (182 + STACK_EXTRA)
#define STACK_RX_SMARTPORT (190 + STACK_EXTRA)
#define STACK_RX_JETIEX (240 + STACK_EXTRA)
#define STACK_RX_SBUS (220 + STACK_EXTRA)
#define STACK_RX_HITEC (200 + STACK_EXTRA)
#define STACK_RX_XBUS (200 + STACK_EXTRA)
#define STACK_RX_SRXL (200 + STACK_EXTRA)
#define STACK_RX_SRXL2 (300 + STACK_EXTRA)
#define STACK_SERIAL_MONITOR (250 + STACK_EXTRA)
#define STACK_RX_CRSF (200 + STACK_EXTRA)
#define STACK_RX_HOTT (300 + STACK_EXTRA)
#define STACK_RX_SANWA (200 + STACK_EXTRA)

#define STACK_SIM_RX (160 + STACK_EXTRA)

#define STACK_SEND_SBUS_PACKET (700 + STACK_EXTRA)

#define STACK_SENSOR_FRSKY_D (164 + STACK_EXTRA)
#define STACK_SENSOR_FRSKY_D_CELL (158 + STACK_EXTRA)
#define STACK_SENSOR_SMARTPORT (180 + STACK_EXTRA)
#define STACK_SENSOR_SMARTPORT_DOUBLE (166 + STACK_EXTRA)
#define STACK_SENSOR_SMARTPORT_COORDINATE (164 + STACK_EXTRA)
#define STACK_SENSOR_SMARTPORT_DATETIME (164 + STACK_EXTRA)
#define STACK_SENSOR_SMARTPORT_CELL (166 + STACK_EXTRA)
#define STACK_SMARTPORT_PACKET_TASK (160 + STACK_EXTRA)
#define STACK_SMARTPORT_SENSOR_VOID_TASK (166 + STACK_EXTRA)

#define STACK_ESC_HW3 (168 + STACK_EXTRA)
#define STACK_ESC_HW4 (250 + STACK_EXTRA)
#define STACK_ESC_HW5 (300 + STACK_EXTRA)
#define STACK_ESC_PWM (160 + STACK_EXTRA)
#define STACK_ESC_CASTLE (500 + STACK_EXTRA)
#define STACK_ESC_KONTRONIK (212 + STACK_EXTRA)
#define STACK_ESC_APD_F (194 + STACK_EXTRA)
#define STACK_ESC_APD_HV (194 + STACK_EXTRA)
#define STACK_GPS (202 + STACK_EXTRA)

#define STACK_VOLTAGE (156 + STACK_EXTRA)
#define STACK_CURRENT (166 + STACK_EXTRA)
#define STACK_NTC (160 + STACK_EXTRA)
#define STACK_AIRSPEED (158 + STACK_EXTRA)

#define STACK_BMP280 (228 + STACK_EXTRA)
#define STACK_MS5611 (172 + STACK_EXTRA)
#define STACK_BMP180 (174 + STACK_EXTRA)

#define STACK_VSPEED (152 + STACK_EXTRA)
#define STACK_DISTANCE (152 + STACK_EXTRA)
#define STACK_CELL_COUNT (180 + STACK_EXTRA)
#define STACK_AUTO_OFFSET (140 + STACK_EXTRA)
#define STACK_PWM_OUT (200 + STACK_EXTRA)

#define STACK_USB (180 + STACK_EXTRA)
#define STACK_LED (186 + STACK_EXTRA)

/* RPM multiplier */
#define RPM_MULTIPLIER (RPM_PINION_TEETH / (1.0 * RPM_MAIN_TEETH * RPM_PAIR_OF_POLES))

/* Averaging elements to alpha */
#define ALPHA(ELEMENTS) (2.0 / ((ELEMENTS) + 1))
/* Averaging alpha to elements */
#define ELEMENTS(ALPHA) (uint) round((2.0 / (ALPHA) - 1))

#endif
