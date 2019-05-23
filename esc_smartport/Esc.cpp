#include "Esc.h"

// Pwm in INT0
volatile uint16_t pwmInInit = 0;
volatile uint16_t pwmInLenght = 0;
volatile uint8_t isr_int0;

// Pwm out TIMER1
volatile uint16_t pwmOutLow;
volatile uint16_t pwmOutHigh;
volatile bool pwmOutState = false;
volatile bool pwmOutActive = false;
volatile uint8_t isr_timer;

// Castle INT1
volatile uint16_t castlelinkInit = 0;
volatile uint16_t castlelinkLenght = 0;
volatile uint8_t cont = 0;

// Isr telemetry
volatile IsrTelemetry isrTelemetry;

ISR(INT0_vect) {
  switch (isr_int0) {
  case INT0_PWM_IN:
    pwmInLenght = (uint16_t)micros() - pwmInInit;
    pwmInInit = micros();
    break;
  case INT0_RX: // pin rx

    if (!digitalRead(PIN_PWM_IN_RX)) {

      pwmOutLow = ((uint16_t)micros() - pwmInInit) * (uint32_t)F_CPU / 8000000;

      pwmOutHigh = F_CPU / 400 - pwmOutLow;
    }
    pwmInInit = micros();
    break;
  }
}

ISR(INT1_vect) {
  castlelinkLenght = (uint16_t)micros() - castlelinkInit;
  castlelinkInit = micros();
  if (castlelinkLenght > 19800) {
    cont = 0;
  } else if (castlelinkLenght < 10000) {
    switch (cont) {
      case 0:
        isrTelemetry.ms1 = castlelinkLenght;
        break;
      case 1:
        isrTelemetry.voltage = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 20) / (2 * isrTelemetry.ms);
        if (isrTelemetry.voltage < 0) isrTelemetry.voltage = 0;
        break;
      case 2:
        isrTelemetry.rippleVoltage = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 4) / (2 * isrTelemetry.ms);
        if (isrTelemetry.rippleVoltage < 0) isrTelemetry.rippleVoltage = 0;
        break;
      case 3:
        isrTelemetry.current = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 50) / (2 * isrTelemetry.ms);
        if (isrTelemetry.current < 0) isrTelemetry.current = 0;
        break;
      case 6:
        isrTelemetry.rpm = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 20416.7) / (2 * isrTelemetry.ms);
        if (isrTelemetry.rpm < 0) isrTelemetry.rpm = 0;
        break;
      case 7:
        isrTelemetry.becVoltage = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 4) / (2 * isrTelemetry.ms);
        if (isrTelemetry.becVoltage < 0) isrTelemetry.becVoltage = 0;
        break;
      case 8:
        isrTelemetry.becCurrent = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 4) / (2 * isrTelemetry.ms);
        if (isrTelemetry.becCurrent < 0) isrTelemetry.becCurrent = 0;
        break;
      case 9:
        if (castlelinkLenght > 700) {
          isrTelemetry.temperature = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 30) / (2 * isrTelemetry.ms);
        } else {
          isrTelemetry.ms = (isrTelemetry.ms1 + 2 * castlelinkLenght) / 2;
          cont--;
        }
        break;
      case 10:
        if (castlelinkLenght > 700) {
          isrTelemetry.temperature = ((2 * (float)castlelinkLenght - isrTelemetry.ms) * 30) / (2 * isrTelemetry.ms);;
        } else {
          isrTelemetry.ms = (isrTelemetry.ms1 + 2 * castlelinkLenght) / 2;
        }
        break;
    }
    cont++;
  }
}

ISR(TIMER1_COMPA_vect) {
  switch (isr_timer) {
  case TIMER_PWM_OUT:
    if (pwmOutActive) {
      if (pwmOutState) {
        digitalWrite(PIN_PWM_OUT, LOW);
        pwmOutState = false;
        OCR1A = pwmOutLow;
      } else {
        digitalWrite(PIN_PWM_OUT, HIGH);
        pwmOutState = true;
        OCR1A = pwmOutHigh;
      }
    }
    break;
  case TIMER_CASTLE:
    if (pwmOutState) {
      pinMode(PIN_CASTLE, OUTPUT);
      digitalWrite(PIN_CASTLE, LOW);
      pwmOutState = false;
      OCR1A = pwmOutLow;
    } else {
      pinMode(PIN_CASTLE, INPUT);
      pwmOutState = true;
      OCR1A = pwmOutHigh;
    }
    break;
  }
}

Esc::Esc(Stream &serial) : _serial(serial) {}

bool Esc::readHWV3() {
  static uint16_t tsEsc = 0;
  while (_serial.available() >= 10) {
    if (_serial.read() == 0x9B) {
      uint8_t data[9];
      uint8_t cont = _serial.readBytes(data, 64);
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
        rpm = (float)((uint32_t)data[7] << 16 | data[8] << 8 | data[9]);
        voltage = (float)((uint16_t)(data[10] << 8 | data[11])) / 100;
        temp1 = (float)((uint16_t)(data[14] << 8 | data[15])) / 100;
        temp2 = (float)((uint16_t)(data[16] << 8 | data[17])) / 100;
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
      noInterrupts();
      pwmOutLow =
          (uint16_t)((float)6.225 * F_CPU / rpm); // (Hz/scaler)*60/rpm*0.83
      pwmOutHigh =
          (uint16_t)((float)1.275 * F_CPU / rpm); // (Hz/scaler)*60/rpm*0.17
      pwmOutActive = true;
      interrupts();
    }
    else {
      noInterrupts();
      pwmOutLow = 5000;
      pwmOutHigh = 5000;
      pwmOutActive = false;
      digitalWrite(PIN_PWM_OUT, LOW);
      interrupts();
    }
  }
  return statusChange;
}

void Esc::setProtocol(uint8_t protocol) {
  _protocol = protocol;
  switch (_protocol) {
  case PROTOCOL_PWM:
    isr_int0 = INT0_PWM_IN;
    pinMode(PIN_PWM_IN_RX, INPUT);
    noInterrupts();
    DDRD &= ~(1 << DDD2);   // PIN 2 input
    PORTD |= (1 << PORTD2); // Pull up
    EIMSK |= (1 << INT0);   // Enable external interrupt INT0 (PIN 2)
    EICRA |= (1 << ISC00);  // Trigger INT0 on rising edge
    EICRA |= (1 << ISC01);
    EIMSK |= (0 << INT1);
    TIMSK1 = 0;
    interrupts();
    break;
  case PROTOCOL_CASTLE:
    isr_timer = TIMER_CASTLE;
    isr_int0 = INT0_RX;
    pinMode(PIN_CASTLE, INPUT);
    noInterrupts();
    // Castle interrupt (pin 3)
    DDRD &= ~(1 << DDD3);   // PIN 3 input
    PORTD |= (1 << PORTD3); // Pull up PIN 3
    EICRA |= (1 << ISC10);  // Trigger INT1 on xx edge
    EICRA |= (1 << ISC11);  // Trigger INT1 on xx edge
    // EIMSK |= (0 << INT1);   // Disable external interrupt INT1 (PIN 3)
    EIMSK |= (1 << INT1); // Enable external interrupt INT1 (PIN 3)

    // Rx interrupt (pin 2)
    pinMode(PIN_PWM_IN_RX, INPUT);
    DDRD &= ~(1 << DDD2);   // PIN 2 input
    PORTD |= (1 << PORTD2); // Pull up PIN 2
    EICRA |= (1 << ISC00);  // Trigger INT0 on change
    EICRA |= (0 << ISC01);  // Trigger INT0 on change
    // EIMSK |= (0 << INT0);   // Disable external interrupt INT0 (PIN 2)
    EIMSK |= (1 << INT0); // Enable external interrupt INT0 (PIN 2)

    // Timer interrupt (pwm out)
    pwmOutHigh = F_CPU / 8000 * 19;
    pwmOutLow = F_CPU / 8000;
    // CTC mode
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    // scaler 8
    TCCR1B |= 1 << CS11;
    // cont = 0
    TCNT1 = 0;
    // comp
    OCR1A = pwmOutHigh;
    // init comp
    TIMSK1 |= (1 << OCIE1A);
    interrupts();

    break;

  default:
    noInterrupts();
    EIMSK |= (0 << INT0);
    EIMSK |= (0 << INT1);
    if (_pwmOut == false)
      TIMSK1 = 0;
    interrupts();
  }
}

void Esc::setPwmOut(uint8_t pwmOut) {
  _pwmOut = pwmOut;
  if (_pwmOut) {
    isr_timer = TIMER_PWM_OUT;
    pinMode(PIN_PWM_OUT, OUTPUT);
    digitalWrite(PIN_PWM_OUT, LOW);
    noInterrupts();
    // CTC mode
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    // scaler 8
    TCCR1B |= 1 << CS11;
    // cont = 0
    TCNT1 = 0;
    // comp
    OCR1A = 5000;
    // init comp
    TIMSK1 |= (1 << OCIE1A);
    interrupts();

  } else if (_protocol != PROTOCOL_CASTLE)
    TIMSK1 = 0;
}

float Esc::getRpm() { return rpm; }

float Esc::getVolt() { return voltage; }

float Esc::getTemp1() { return temp1; }

float Esc::getTemp2() { return temp2; }

float Esc::getRippleVoltage() { return rippleVoltage; }

float Esc::getCurrent() { return current; }

float Esc::getBecVoltage() { return becVoltage; }

float Esc::getBecCurrent() { return becCurrent; }
