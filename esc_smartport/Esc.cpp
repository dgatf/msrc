#include "Esc.h"

// Pwm in: TIMER 1 CAPT, PIN 8
volatile uint16_t pwmInLenght = 60000;
volatile uint32_t tsPwmIn = 0;

ISR(TIMER1_CAPT_vect) {
  volatile static uint16_t pwmInInit = 0;
  if (ICR1 - pwmInInit > 0) {
    pwmInLenght = ICR1 - pwmInInit;
    pwmInInit = ICR1;
    tsPwmIn = micros();
  }
}

Esc::Esc(Stream &serial) : _serial(serial) {}

bool Esc::readHWV3() {
  static uint16_t tsEsc = 0;
  while (_serial.available() >= 10) {
    if (_serial.read() == 0x9B) {
      uint8_t data[9];
      uint8_t cont = _serial.readBytes(data, 9);
      if (cont == 9 && data[3] == 0 && data[5] == 0) {
        uint16_t rpmCycle = (uint16_t)data[7] << 8 | data[8];
        rpm = (float)60000000UL / rpmCycle;
        tsEsc = millis();
#ifdef DEBUG_ESC
        uint32_t pn =
            (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
        _serial.print("PN: ");
        _serial.print(pn);
        _serial.print(" RPM: ");
        _serial.println(rpm);
#endif
        return true;
      }
    }
  }

  if (rpm == 0)
    return false;

  if ((uint16_t)millis() - tsEsc > 150) {
    rpm = 0;
    return true;
  }

  return false;
}

bool Esc::readHWV4() {
  while (_serial.available() >= 19) {

    uint8_t header = _serial.read();

    // short packet
    if (header == 0x9B) {
      uint8_t data[18];
      uint8_t cont = _serial.readBytes(data, 18);
      if (cont == 18 && data[0] != 0x9B) {
        rpm = (uint32_t)data[7] << 16 | (uint16_t)data[8] << 8 | data[9];
        voltage = (float)((uint16_t)data[10] << 8 | data[11]) / 89;
        current = (float)((uint16_t)data[12] << 8 | data[13]) / 100;
        temp1 = (float)((uint16_t)data[14] << 8 | data[15]) / 100;
        temp2 = (float)((uint16_t)data[16] << 8 | data[17]) / 100;
#ifdef DEBUG_ESC
        uint32_t pn =
            (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
        _serial.print("PN: ");
        _serial.print(pn);
        _serial.print(" RPM: ");
        _serial.print(rpm);
        _serial.print(" Volt: ");
        _serial.print(voltage);
        _serial.print(" Curr: ");
        _serial.print(current);
        _serial.print(" Temp1: ");
        _serial.print(temp1);
        _serial.print(" Temp2: ");
        _serial.print(temp2);
        /*_serial.print("  ");
        for (uint8_t i = 0; i < cont; i++) {
          _serial.print(data[i], HEX);
          _serial.print(" ");
        }*/
        _serial.println();
#endif
        return true;
      }
    }

    // long packet
    if (header == 0xB9) {
      uint8_t data[32];
      uint8_t cont = _serial.readBytes(data, 32);
      if (cont == 32 && data[0] == 0x9B) {
        rpm = (uint32_t)data[21] << 16 | (uint16_t)data[22] << 8 | data[23];
        voltage = (float)((uint16_t)data[24] << 8 | data[25]) / 100;
        current = (float)((uint16_t)data[26] << 8 | data[27]) / 100;
        temp1 = (float)((uint16_t)data[28] << 8 | data[29]) / 100;
        temp2 = (float)((uint16_t)data[30] << 8 | data[31]) / 100;
#ifdef DEBUG_ESC
        uint32_t pn =
            (uint32_t)data[14] << 16 | (uint16_t)data[15] << 8 | data[16];
        _serial.print("PN: ");
        _serial.print(pn);
        _serial.print(" RPM: ");
        _serial.print(rpm);
        _serial.print(" Volt: ");
        _serial.print(voltage);
        _serial.print(" Curr: ");
        _serial.print(current);
        _serial.print(" Temp1: ");
        _serial.print(temp1);
        _serial.print(" Temp2: ");
        _serial.print(temp2);
        /*_serial.print("  ");
        for (uint8_t i = 0; i < cont; i++) {
          _serial.print(data[i], HEX);
          _serial.print(" ");
        }*/
        _serial.println();
#endif
        return true;
      }
    }
  }
  return false;
}

void Esc::readPWM() {
  static uint8_t cont = 0;
  if (pwmInLenght > 0 &&
      pwmInLenght * COMP_TO_MICROS < PWM_IN_TRIGGER_MICROS &&
      micros() - tsPwmIn < PWM_IN_TRIGGER_MICROS) {
    if (cont > PWM_IN_TRIGGER_PULSES) {
      rpm = 60000000UL / pwmInLenght * COMP_TO_MICROS;
    }
    if (cont <= PWM_IN_TRIGGER_PULSES) cont++;
#ifdef DEBUG_ESC
    Serial.print("RPM: ");
    Serial.println(rpm);
#endif
  } else {
    rpm = 0;
    cont = 0;
  }
}

bool Esc::read() {
  bool statusChange = false;
  switch (_protocol) {
  case PROTOCOL_HW_V3:
    statusChange = readHWV3();
    break;
  case PROTOCOL_HW_V4:
    statusChange = readHWV4();
    break;
  case PROTOCOL_PWM:
    readPWM();
    statusChange = true;
    break;
  }
  return statusChange;
}

void Esc::setProtocol(uint8_t protocol) {
  _protocol = protocol;
  switch (_protocol) {
  case PROTOCOL_PWM:
    // TIMER1,capture ext int, scaler 8. PIN 8
    TCCR1A = 0;
    TCCR1B = _BV(CS11);
    TIMSK1 = _BV(ICIE1);
    break;
  default:
    TIMSK1 = 0;
  }
}

float Esc::getRpm() { return rpm; }

float Esc::getVolt() { return voltage; }

float Esc::getTemp1() { return temp1; }

float Esc::getTemp2() { return temp2; }

float Esc::getCurrent() { return current; }
