/*
 *            Multi Sensor RC - MSRC
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 *           Daniel.GeA.1000@gmail.com
 *
 */


#include "config.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#if (defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"

#if RX_PROTOCOL == RX_SMARTPORT
#include "smartport.h"
#endif
#if RX_PROTOCOL == RX_XBUS
#include "xbus.h"
#endif
#if RX_PROTOCOL == RX_SRXL
#include "srxl.h"
#endif
#if RX_PROTOCOL == RX_FRSKY
#include "frsky_d.h"
#endif
#if RX_PROTOCOL == RX_IBUS
#include "ibus.h"
#endif
#if RX_PROTOCOL == RX_SBUS
#include "sbus.h"
#endif
#if RX_PROTOCOL == RX_MULTIPLEX
#include "multiplex.h"
#endif

PwmOut pwmOut;

#if RX_PROTOCOL == RX_SMARTPORT
#if !defined(__MKL26Z64__) && !defined(__MK20DX128__) && !defined(__MK20DX256__) && !defined(__MK64FX512__) && !defined(__MK66FX1M0__) && !defined(__IMXRT1062__) && !defined(SMARTPORT_FRSKY_HARDWARE_SERIAL)
SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX, true);
#endif
Smartport smartport(SMARTPORT_FRSKY_SERIAL);
#endif

#if RX_PROTOCOL == RX_XBUS
Xbus xbus;
#endif

#if RX_PROTOCOL == RX_SRXL
Srxl srxl(SRXL_IBUS_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_FRSKY
#if !defined(__MKL26Z64__) && !defined(__MK20DX128__) && !defined(__MK20DX256__) && !defined(__MK64FX512__) && !defined(__MK66FX1M0__) && !defined(__IMXRT1062__) && !defined(SMARTPORT_FRSKY_HARDWARE_SERIAL)
SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX, true);
#endif
Frsky frsky(SMARTPORT_FRSKY_SERIAL);
#endif

#if RX_PROTOCOL == RX_IBUS
Ibus ibus(SRXL_IBUS_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_SBUS
Sbus sbus(SRXL_IBUS_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_MULTIPLEX
Multiplex multiplex(SRXL_IBUS_SBUS_SERIAL);
#endif

void setup();
void loop();