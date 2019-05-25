#include "Esc.h"

// Pwm in: TIMER 1 CAPT, PIN 8
volatile uint16_t pwmInLenght;

// Rx in: EXT INT0
volatile uint16_t rxLenght;

// Castle INT1
volatile uint16_t castleInit = 0;
volatile uint16_t castleLenght;
volatile uint8_t cont = 0;

// TIMER2: disable PCINT for Rx
volatile uint8_t contTimer2 = 0;
volatile uint8_t  _PCMSK0;
volatile uint8_t  _PCMSK1;
volatile uint8_t  _PCMSK2;

// Isr telemetry
volatile IsrTelemetry isrTelemetry;

ISR(TIMER1_CAPT_vect) {
  static uint16_t pwmInInit = 0;
  pwmInLenght = COMP_TO_MS * (ICR1 - pwmInInit);
  pwmInInit = ICR1;
}

ISR(TIMER2_OVF_vect) {
  if (contTimer2 == 1) {
    _PCMSK0 = PCMSK0;
    _PCMSK1 = PCMSK1;
    _PCMSK2 = PCMSK2;
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = 0;
    contTimer2 = 0;
  } else contTimer2++;
}

ISR(INT0_vect) {
  static uint16_t rxInit = 0;
  if (digitalRead(PIN_RX)) {
    rxInit = micros();
    TIMSK2 = 1 << TOIE2;
  } else {
    rxLenght = micros()-rxInit;
    OCR1A = rxLenght / COMP_TO_MS;
    PCMSK0 = _PCMSK0;
    PCMSK1 = _PCMSK0;
    PCMSK2 = _PCMSK0;
    TIMSK2 = 0;
  }
}

ISR(INT1_vect) {
  pinMode(9, INPUT);
  castleLenght = micros() - castleInit;
  castleInit = micros();
  if (castleLenght > 19500)
    cont = 0;
  if (castleLenght < 7000) {
    switch (cont) {
    case 0:
      isrTelemetry.ms1 = castleLenght;
      break;
    case 1:
      isrTelemetry.voltage =
          (float)((2 * castleLenght - isrTelemetry.ms) * 20) /
          (2 * isrTelemetry.ms);
      break;
    case 2:
      isrTelemetry.rippleVoltage =
          (float)((2 * castleLenght - isrTelemetry.ms) * 4) /
          (2 * isrTelemetry.ms);
      break;
    case 3:
      isrTelemetry.current =
          (float)((2 * castleLenght - isrTelemetry.ms) * 50) /
          (2 * isrTelemetry.ms);
      break;
    case 6:
      isrTelemetry.rpm =
          (float)((2 * castleLenght - isrTelemetry.ms) * 20416.7) /
          (2 * isrTelemetry.ms);
      break;
    case 7:
      isrTelemetry.becVoltage =
          (float)((2 * castleLenght - isrTelemetry.ms) * 4) /
          (2 * isrTelemetry.ms);
      break;
    case 8:
      isrTelemetry.becCurrent =
          (float)((2 * castleLenght - isrTelemetry.ms) * 4) /
          (2 * isrTelemetry.ms);
      break;
    case 9:
      if (castleLenght > 700) {
        isrTelemetry.temperature =
            (float)((2 * castleLenght - isrTelemetry.ms) * 30) /
            (2 * isrTelemetry.ms);
      } else {
        isrTelemetry.ms = (isrTelemetry.ms1 + 2 * castleLenght) / 2;
        cont--;
      }
      break;
    case 10:
      if (castleLenght > 700) {
        isrTelemetry.temperature =
            (float)((2 * castleLenght - isrTelemetry.ms) * 30) /
            (2 * isrTelemetry.ms);
      } else {
        isrTelemetry.ms = (isrTelemetry.ms1 + 2 * castleLenght) / 2;
      }
      break;
    }
    cont++;
  }
  TCNT2 = 0;
  TIMSK2 = 1 << OCIE2A;
}

ISR(TIMER1_COMPB_vect) { pinMode(PIN_CASTLE, OUTPUT); }

Esc::Esc(Stream &serial) : _serial(serial) {
  _PCMSK0 = PCMSK0;
  _PCMSK1 = PCMSK1;
  _PCMSK2 = PCMSK2;
}

bool Esc::readHWV3() {
  static uint16_t tsEsc = 0;
  while (_serial.available() >= 10) {
    if (_serial.read() == 0x9B) {
      uint8_t data[9];
      uint8_t cont = _serial.readBytes(data, 9);
      if (cont == 9 && data[3] == 0 && data[5] == 0) {
        uint16_t rpmCycle = data[7] << 8 | data[8];
        rpm = (float)60000000 / rpmCycle;
        tsEsc = millis();
#ifdef DEBUG
        uint32_t pn = (uint32_t)data[0] << 16 | data[1] << 8 | data[2];
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
    if (_serial.read() == 0x9B) {
      uint8_t data[18];
      uint8_t cont = _serial.readBytes(data, 18);
      if (cont == 18 && data[1] != 155) {
        rpm = (uint32_t)data[7] << 16 | data[8] << 8 | data[9];
        voltage = (float)(data[10] << 8 | data[11]) / 100;
        temp1 = (float)(data[14] << 8 | data[15]) / 100;
        temp2 = (float)(data[16] << 8 | data[17]) / 100;
#ifdef DEBUG
        uint32_t pn = (uint32_t)data[0] << 16 | data[1] << 8 | data[2];
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
#endif
        return true;
      }
    }
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

bool Esc::readCastle() {
  voltage = isrTelemetry.voltage;
  rippleVoltage = isrTelemetry.rippleVoltage;
  current = isrTelemetry.current;
  rpm = isrTelemetry.rpm;
  becVoltage = isrTelemetry.becVoltage;
  becCurrent = isrTelemetry.becCurrent;
  temp1 = isrTelemetry.temperature;
  return true;
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
  case PROTOCOL_CASTLE:
    statusChange = readCastle();
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

    TIMSK2 = 0;
    EIMSK = 0;
    //PCMSK0 = _PCMSK0;
    //PCMSK1 = _PCMSK1;
    //PCMSK2 = _PCMSK2;
    break;
  case PROTOCOL_CASTLE:
    pinMode(PIN_RX, INPUT);
    pinMode(PIN_CASTLE, INPUT);

    // TIMER1, PWM ICR, scaler 8
    TCCR1A = 1 << COM1A1 | 1 << WGM11;
    TCCR1B = 1 << WGM13 | 1 << WGM12 | 1 << CS11;
    ICR1 = 20000 * F_CPU_SCALER;
    OCR1A = 19000 * F_CPU_SCALER;
    OCR1B = 10000 * F_CPU_SCALER;
    TIMSK1 = 1 << OCIE1B;

    // EXT PIN INT
    DDRD &= ~(1 << DDD3);   // PIN 3 input
    PORTD |= (1 << PORTD3); // Pull up PIN 3
    EICRA |= (1 << ISC10);  // Trigger INT1 on rising edge
    EICRA |= (1 << ISC11);  // Trigger INT1 on rising edge
    // EIMSK |= (0 << INT1);   // Disable external interrupt INT1 (PIN 3)
    EIMSK |= (1 << INT1); // Enable external interrupt INT1 (PIN 3)

    // TIMER2
    TCCR2A = 0;
    TCCR2B =  1 << CS22 | 1 << CS21 | 1 << CS20;
    TIMSK2 = 1 << TOIE2;

    // Rx interrupt (pin 2)
    DDRD &= ~(1 << DDD2);   // PIN 2 input
    PORTD |= (1 << PORTD2); // Pull up PIN 2
    EICRA |= (1 << ISC00);  // Trigger INT0 on change
    EICRA |= (0 << ISC01);  // Trigger INT0 on change
    // EIMSK |= (0 << INT0);   // Disable external interrupt INT0 (PIN 2)
    EIMSK |= (1 << INT0); // Enable external interrupt INT0 (PIN 2)

    break;

  default:
    EIMSK = 0 ;
    TIMSK2 = 0;
    //PCMSK0 = _PCMSK0;
    //PCMSK1 = _PCMSK1;
    //PCMSK2 = _PCMSK2;
    if (_pwmOut == false)
      TIMSK1 = 0;
  }
}

void Esc::setPwmOut(uint8_t pwmOut) {
  _pwmOut = pwmOut;
  if (_pwmOut) {
    // Init pin
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    // Set timer1: WGM mode 14, scaler 8 (pin 9)
    TCCR1A = 1 << WGM11;
    TCCR1B = 1 << WGM13 | 1 << WGM12 | 1 << CS11;
  } else // if (_protocol != PROTOCOL_CASTLE)
    TCCR1A &= ~(1 << COM1A1);
}

float Esc::getRpm() { return rpm; }

float Esc::getVolt() { return voltage; }

float Esc::getTemp1() { return temp1; }

float Esc::getTemp2() { return temp2; }

float Esc::getRippleVoltage() { return rippleVoltage; }

float Esc::getCurrent() { return current; }

float Esc::getBecVoltage() { return becVoltage; }

float Esc::getBecCurrent() { return becCurrent; }
