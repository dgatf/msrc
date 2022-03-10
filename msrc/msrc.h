/*
 *            Multi Sensor RC - MSRC
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 *           Daniel.GeA.1000@gmail.com
 *
 */

#include "constants.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "softserial.h"
#include "hardserial.h"
#include "serial.h"
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "escApdF.h"
#include "escApdHV.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"

#include "smartport.h"
#include "xbus.h"
#include "srxl.h"
#include "frsky_d.h"
#include "ibus.h"
#include "sbus.h"
#include "multiplex.h"
#include "jetiex.h"
#include "hitec.h"

#if RX_PROTOCOL == RX_SMARTPORT
Smartport smartport(SMARTPORT_FRSKY_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_XBUS
Xbus xbus;
#endif

#if RX_PROTOCOL == RX_SRXL
Srxl srxl(SRXL_IBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_FRSKY
Frsky frsky(SMARTPORT_FRSKY_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_IBUS
Ibus ibus(SRXL_IBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_SBUS
Sbus sbus(SMARTPORT_FRSKY_SBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_MULTIPLEX
Multiplex multiplex(SRXL_IBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_JETIEX
JetiEx jetiEx(SRXL_IBUS_SERIAL);
#endif

#if RX_PROTOCOL == RX_HITEC
Hitec hitec;
#endif

PwmOut pwmOut;

void setup();
void loop();