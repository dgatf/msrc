/*
 *            Multi Sensor RC - MSRC
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 *           Daniel.GeA.1000@gmail.com
 *
 */


#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>

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
#include "config.h"
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

PwmOut pwmOut;

#if RX_PROTOCOL == RX_SMARTPORT
#if !(defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))
SoftwareSerial SMARTPORT_FRSKY_SERIAL(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX, true);
#endif
Smartport smartport(SMARTPORT_FRSKY_SERIAL);
#endif

#if RX_PROTOCOL == RX_XBUS
Xbus xbus;
#endif

#if RX_PROTOCOL == RX_SRXL
Srxl srxl(SRXL_IBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_FRSKY
#if !(defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))
SoftwareSerial SMARTPORT_FRSKY_SERIAL(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX, true);
#endif
Frsky frsky(SMARTPORT_FRSKY_SERIAL);
#endif

#if RX_PROTOCOL == RX_IBUS
Ibus ibus(SRXL_IBUS_SERIAL);
#endif

void setup();
void loop();