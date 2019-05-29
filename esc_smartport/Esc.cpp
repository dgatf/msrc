#include "Esc.h"

// Pwm in: TIMER 1 CAPT, PIN 8
volatile uint16_t pwmInLenght;

ISR(TIMER1_CAPT_vect) {
  volatile static uint16_t pwmInInit = 0;
  pwmInLenght = COMP_TO_MS * (ICR1 - pwmInInit);
  pwmInInit = ICR1;
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
        rpm = (float)60000000 / rpmCycle;
        tsEsc = millis();
#ifdef DEBUG
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
  if (_serial.available() >= 19) {
    uint8_t data[33];
    uint8_t cont = _serial.readBytes(data, 33);
    if (cont == 19 && data[1] != 155) {
      rpm = (uint32_t)data[8] << 16 | (uint16_t)data[9] << 8 | data[10];
      voltage = (float)((uint16_t)data[11] << 8 | data[12]) / 100;
      temp1 = (float)((uint16_t)data[15] << 8 | data[16]) / 100;
      temp2 = (float)((uint16_t)data[17] << 8 | data[18]) / 100;

#ifdef DEBUG
      uint32_t pn = (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
      _serial.print("PN: ");
      _serial.print(pn);
      _serial.print(" RPM: ");
      _serial.print(rpm);
      _serial.print(" Volt: ");
      _serial.print(voltage);
      _serial.print(" Temp1: ");
      _serial.print(temp1);
      _serial.print(" Temp2: ");
      _serial.print(temp2);
      _serial.print("  ");
      for (uint8_t i = 0; i < cont; i++) {
        _serial.print(data[i]);
        _serial.print(" ");
      }
      _serial.println();
#endif
      return true;
    }
    if (cont == 33) {
      rpm = (uint32_t)data[22] << 16 | (uint16_t)data[23] << 8 | data[24];
      voltage = (float)((uint16_t)data[25] << 8 | data[26]) / 100;
      temp1 = (float)((uint16_t)data[29] << 8 | data[30]) / 100;
      temp2 = (float)((uint16_t)data[31] << 8 | data[32]) / 100;
#ifdef DEBUG
      uint32_t pn =
          (uint32_t)data[15] << 16 | (uint16_t)data[16] << 8 | data[17];
      _serial.print("PN: ");
      _serial.print(pn);
      _serial.print(" RPM: ");
      _serial.print(rpm);
      _serial.print(" Volt: ");
      _serial.print(voltage);
      _serial.print(" Temp1: ");
      _serial.print(temp1);
      _serial.print(" Temp2: ");
      _serial.println(temp2);
      _serial.print("  ");
      for (uint8_t i = 0; i < cont; i++) {
        _serial.print(data[i]);
        _serial.print(" ");
      }
      _serial.println();
#endif
      return true;
    }
#ifdef DEBUG
    if (!(cont == 19 && data[1] != 155) && cont != 33) {
      for (uint8_t i = 0; i < cont; i++) {
        _serial.print(data[i]);
        _serial.print(" ");
      }
      _serial.println();
    }
#endif
  }
  return false;
}

void Esc::readPWM() {
  if (pwmInLenght < 15000) {
    rpm = (float)60000000 / pwmInLenght;
  } else {
    rpm = 0;
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
  if (_pwmOut == true && _protocol != PROTOCOL_PWM && statusChange) {
    if (rpm >= 2000) {
      TCCR1A |= 1 << COM1A1;
      ICR1 = 7.5 * (uint32_t)F_CPU / rpm;
      OCR1A = 0.17 * ICR1;
    } else {
      TCCR1A &= ~(1 << COM1A1);
    }
  }
  return statusChange;
}

void Esc::setProtocol(uint8_t protocol) {
  _protocol = protocol;
  switch (_protocol) {
  case PROTOCOL_PWM:
    // TIMER1,capture ext int, scaler 8. PIN 8
    TCCR1A = 0;
    TCCR1B = 1 << CS11;
    TIMSK1 = 1 << ICIE1;
    break;
  default:
    if (_pwmOut == false)
      TIMSK1 = 0;
  }
}

void Esc::setPwmOut(uint8_t pwmOut) {
  _pwmOut = pwmOut;
  if (_pwmOut) {
    // Init pin
    pinMode(PIN_PWM_OUT, OUTPUT);
    digitalWrite(PIN_PWM_OUT, LOW);
    // Set timer1: WGM mode 14, scaler 8 (pin 9)
    TCCR1A = 1 << WGM11;
    TCCR1B = 1 << WGM13 | 1 << WGM12 | 1 << CS11;
  } else
    TCCR1A &= ~(1 << COM1A1);
}

float Esc::getRpm() { return rpm; }

float Esc::getVolt() { return voltage; }

float Esc::getTemp1() { return temp1; }

float Esc::getTemp2() { return temp2; }

float Esc::getCurrent() { return current; }
