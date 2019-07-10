#include "esc_smartport.h"

void readConfig() {
  uint32_t chk;
  EEPROM.get(0, chk);
  if (chk == 0x64616E69) {
    EEPROM.get(4, config);
  } else {
    writeConfig();
  }
#ifdef DEBUG
  escSerial.println("Read config");
  escSerial.print("Protocol: ");
  escSerial.println(config.protocol);
  escSerial.print("Voltage1: ");
  escSerial.println(config.voltage1);
  escSerial.print("Voltage2: ");
  escSerial.println(config.voltage2);
  escSerial.print("Current: ");
  escSerial.println(config.current);
  escSerial.print("NTC1: ");
  escSerial.println(config.ntc1);
  escSerial.print("NTC2: ");
  escSerial.println(config.ntc2);
  escSerial.print("Pwm out: ");
  escSerial.println(config.pwmOut);
  escSerial.print("Refresh RPM: ");
  escSerial.println(config.refreshRpm);
  escSerial.print("Refresh Volt: ");
  escSerial.println(config.refreshVolt);
  escSerial.print("Refresh Curr: ");
  escSerial.println(config.refreshCurr);
  escSerial.print("Refresh Temp: ");
  escSerial.println(config.refreshTemp);
  escSerial.print("Queue RPM: ");
  escSerial.println(config.queueRpm);
  escSerial.print("Queue Volt: ");
  escSerial.println(config.queueVolt);
  escSerial.print("Queue Curr: ");
  escSerial.println(config.queueCurr);
  escSerial.print("Queue Temp: ");
  escSerial.println(config.queueTemp);
  escSerial.print("Queue PWM: ");
  escSerial.println(config.queuePwm);
#endif
}

void writeConfig() {
  EEPROM.put(0, (uint32_t)0x64616E69);
  EEPROM.put(4, config);
#ifdef DEBUG
  escSerial.println("Write config");
  escSerial.print("Protocol: ");
  escSerial.println(config.protocol);
  escSerial.print("Voltage1: ");
  escSerial.println(config.voltage1);
  escSerial.print("Voltage2: ");
  escSerial.println(config.voltage2);
  escSerial.print("Current: ");
  escSerial.println(config.current);
  escSerial.print("NTC1: ");
  escSerial.println(config.ntc1);
  escSerial.print("NTC2: ");
  escSerial.println(config.ntc2);
  escSerial.print("Pwm out: ");
  escSerial.println(config.pwmOut);
  escSerial.print("Refresh RPM: ");
  escSerial.println(config.refreshRpm);
  escSerial.print("Refresh Volt: ");
  escSerial.println(config.refreshVolt);
  escSerial.print("Refresh Curr: ");
  escSerial.println(config.refreshCurr);
  escSerial.print("Refresh Temp: ");
  escSerial.println(config.refreshTemp);
  escSerial.print("Queue RPM: ");
  escSerial.println(config.queueRpm);
  escSerial.print("Queue Volt: ");
  escSerial.println(config.queueVolt);
  escSerial.print("Queue Curr: ");
  escSerial.println(config.queueCurr);
  escSerial.print("Queue Temp: ");
  escSerial.println(config.queueTemp);
  escSerial.print("Queue PWM: ");
  escSerial.println(config.queuePwm);
#endif
}

void initConfig() {

  esc.setProtocol(config.protocol);
  setPwmOut();

  smartport.deleteElements();

  telemetry.rpmQ.del(16);
  telemetry.voltageQ.del(16);
  telemetry.temp1Q.del(16);
  telemetry.temp2Q.del(16);
  telemetry.voltageAnalog1Q.del(16);
  telemetry.voltageAnalog2Q.del(16);
  telemetry.currentAnalogQ.del(16);
  telemetry.ntc1Q.del(16);
  telemetry.ntc2Q.del(16);

  switch (config.protocol) {
  case PROTOCOL_HW_V3:
  case PROTOCOL_PWM:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, config.refreshRpm);
    telemetry.rpmQ.initQueue(0, config.queueRpm);
    break;
  case PROTOCOL_HW_V4:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, config.refreshRpm);
    telemetry.rpmQ.initQueue(0, config.queueRpm);
    telemetry.escPowerP =
        smartport.addElement(ESC_POWER_FIRST_ID, config.refreshVolt);
    telemetry.voltageQ.initQueue(0, config.queueVolt);
    telemetry.temp1P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID, config.refreshTemp);
    telemetry.temp1Q.initQueue(0, config.queueTemp);
    telemetry.temp2P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID + 1, config.refreshTemp);
    telemetry.temp2Q.initQueue(0, config.queueTemp);
    break;
  }

  if (config.voltage1 == true) {
    telemetry.voltageAnalog1P =
        smartport.addElement(A3_FIRST_ID, config.refreshVolt);
    telemetry.voltageAnalog1Q.initQueue(0, config.queueVolt);
  }
  if (config.voltage2 == true) {
    telemetry.voltageAnalog2P =
        smartport.addElement(A4_FIRST_ID, config.refreshVolt);
    telemetry.voltageAnalog2Q.initQueue(0, config.queueVolt);
  }
  if (config.current == true) {
    telemetry.currentAnalogP =
        smartport.addElement(CURR_FIRST_ID, config.refreshCurr);
    telemetry.currentAnalogQ.initQueue(0, config.queueCurr);
  }
  if (config.ntc1 == true) {
    telemetry.ntc1P = smartport.addElement(T1_FIRST_ID, config.refreshTemp);
    telemetry.ntc1Q.initQueue(0, config.queueTemp);
  }
  if (config.ntc2 == true) {
    telemetry.ntc2P = smartport.addElement(T2_FIRST_ID, config.refreshTemp);
    telemetry.ntc2Q.initQueue(0, config.queueTemp);
  }
}

void setPwmOut() {
  noInterrupts();
  if (config.pwmOut) {
    // Init pin
    pinMode(PIN_PWM_OUT_OCR, OUTPUT);
    digitalWrite(PIN_PWM_OUT_OCR, LOW);
    pinMode(PIN_PWM_OUT_ICRA, OUTPUT);
    digitalWrite(PIN_PWM_OUT_ICRA, LOW);
#if MODE_PWM_OUT == ICR
    // Set timer1: WGM mode 14 (ICR), scaler 8 (OC1A, PB1, pin 9)
    TCCR1A = _BV(WGM11);
#else
    // Set timer1: WGM mode 15 (OCR), scaler 8 (OC1B, PB2, pin 10)
    TCCR1A = _BV(WGM11) | _BV(WGM10);
#endif
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  } else {
    TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
  }
  interrupts();
}

float readVoltageAnalog(uint8_t pin) {
  const float analogToVolt = (float)BOARD_VCC / 1024;
  uint16_t value = analogRead(pin);
  return value * analogToVolt;
}

float readNtc(uint8_t pin) {
  float volt = readVoltageAnalog(pin);
  float ntcR_Rref = (volt * NTC_R1 / (BOARD_VCC - volt)) / NTC_R_REF;
  /*return
      1 / (NTC_A1 + NTC_B1 * log(ntcR_Rref) + NTC_C1 * pow(log(ntcR_Rref), 2) +
           NTC_D1 * pow(log(ntcR_Rref), 3)) -
      273.15;*/
  float temp = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
  if (temp < 0) return 0;
  return temp;
}

void setup() {
  escSerial.begin(19200);
  while (!escSerial) {
  }
  escSerial.setTimeout(ESCSERIAL_TIMEOUT);
  smartportSerial.begin(57600);
#ifdef DEBUG
  escSerial.println("DEBUG");
  escSerial.print("V");
  escSerial.print(VERSION_MAJOR);
  escSerial.print(".");
  escSerial.println(VERSION_MINOR);
#endif

#ifdef CONFIG_LUA
  readConfig();
#endif

  initConfig();
}

void loop() {

  float valueTelemetry;
  bool statusChange = esc.read();

  switch (config.protocol) {
  case PROTOCOL_HW_V3:
    if (statusChange) {
      valueTelemetry = esc.getRpm() / config.queueRpm;
      telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
      telemetry.rpmQ.enqueue(valueTelemetry);
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);
    }
    break;
  case PROTOCOL_HW_V4:

    if (statusChange) {

      valueTelemetry = esc.getRpm() / config.queueRpm;
      telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
      telemetry.rpmQ.enqueue(valueTelemetry);
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);

      valueTelemetry = esc.getVolt() / config.queueVolt;
      telemetry.voltageAvg += valueTelemetry - telemetry.voltageQ.dequeue();
      telemetry.voltageQ.enqueue(valueTelemetry);
      *telemetry.escRpmConsP =
          smartport.formatEscPower(telemetry.voltageAvg, 0);

      valueTelemetry = esc.getTemp1() / config.queueTemp;
      telemetry.temp1Avg += valueTelemetry - telemetry.temp1Q.dequeue();
      telemetry.temp1Q.enqueue(valueTelemetry);
      *telemetry.escRpmConsP =
          smartport.formatData(ESC_TEMPERATURE_FIRST_ID, telemetry.temp1Avg);

      valueTelemetry = esc.getTemp2() / config.queueTemp;
      telemetry.temp2Avg += valueTelemetry - telemetry.temp2Q.dequeue();
      telemetry.temp2Q.enqueue(valueTelemetry);
      *telemetry.escRpmConsP = smartport.formatData(
          ESC_TEMPERATURE_FIRST_ID + 1, telemetry.temp1Avg);
    }
    break;
  case PROTOCOL_PWM:
    valueTelemetry = esc.getRpm() / config.queueRpm;
    telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
    telemetry.rpmQ.enqueue(valueTelemetry);
    *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);
    break;
  }

  if (config.pwmOut == true && config.protocol != PROTOCOL_PWM && statusChange) {
    telemetry.pwmAvg += esc.getRpm() / config.queuePwm - telemetry.pwmQ.dequeue();
    telemetry.pwmQ.enqueue(esc.getRpm() / config.queuePwm);
    noInterrupts();
    if (esc.getRpm() >= 2000) {
#if MODE_PWM_OUT == ICR
      // ICR
      TCCR1A |= _BV(COM1A1);
      ICR1 = (7.5 * (uint32_t)F_CPU / telemetry.pwmAvg) - 1;
      OCR1A = DUTY * ICR1;
#else
      // OCR
      TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
      OCR1A = (7.5 * (uint32_t)F_CPU / telemetry.pwmAvg) - 1;
      OCR1B = DUTY * OCR1A;
#endif
    } else {
      TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
    }
    interrupts();
  }

  if (config.voltage1 == true) {
    valueTelemetry = readVoltageAnalog(PIN_VOLTAGE1) / config.queueVolt;
    telemetry.voltageAnalog1Avg +=
        valueTelemetry - telemetry.voltageAnalog1Q.dequeue();
    telemetry.voltageAnalog1Q.enqueue(valueTelemetry);
    *telemetry.voltageAnalog1P =
        smartport.formatData(A3_FIRST_ID, telemetry.voltageAnalog1Avg);
  }
  if (config.voltage2 == true) {
    valueTelemetry = readVoltageAnalog(PIN_VOLTAGE2) / config.queueVolt;
    telemetry.voltageAnalog2Avg +=
        valueTelemetry - telemetry.voltageAnalog2Q.dequeue();
    telemetry.voltageAnalog2Q.enqueue(valueTelemetry);
    *telemetry.voltageAnalog2P =
        smartport.formatData(A4_FIRST_ID, telemetry.voltageAnalog2Avg);
  }
  if (config.current == true) {
    valueTelemetry = readVoltageAnalog(PIN_CURRENT) / config.queueCurr;
    telemetry.currentAnalogAvg +=
        valueTelemetry - telemetry.currentAnalogQ.dequeue();
    telemetry.currentAnalogQ.enqueue(valueTelemetry);
    *telemetry.currentAnalogP =
        smartport.formatData(CURR_FIRST_ID, telemetry.currentAnalogAvg);
  }
  if (config.ntc1 == true) {
    valueTelemetry = readNtc(PIN_NTC1) / config.queueTemp;
    telemetry.ntc1Avg += valueTelemetry - telemetry.ntc1Q.dequeue();
    telemetry.ntc1Q.enqueue(valueTelemetry);
    *telemetry.ntc1P = smartport.formatData(T1_FIRST_ID, telemetry.ntc1Avg);
  }
  if (config.ntc2 == true) {
    float valueTelemetry = readNtc(PIN_NTC2) / config.queueTemp;
    telemetry.ntc2Avg += valueTelemetry - telemetry.ntc2Q.dequeue();
    telemetry.ntc2Q.enqueue(valueTelemetry);
    *telemetry.ntc2P = smartport.formatData(T2_FIRST_ID, telemetry.ntc2Avg);
  }

  uint16_t dataId;
  uint32_t value;
  uint8_t type = smartport.processTelemetry(dataId, value);

  if (type == PACKET_RECEIVED) {
    #ifdef DEBUG_TELEMETRY
      Serial.print("Type: ");
      Serial.print(type);
      Serial.print(" DataId: ");
      Serial.print(dataId);
      Serial.print(" Value: ");
      Serial.println(value);
    #endif
    if (dataId == 0x5000) {
      uint32_t value = 0;
      value = config.protocol;
      value |= config.voltage1 << 2;
      value |= config.voltage2 << 3;
      value |= config.current << 4;
      value |= config.ntc1 << 5;
      value |= config.ntc2 << 6;
      value |= config.pwmOut << 7;
      value |= (uint32_t)config.queuePwm << 8;
      value |= (uint32_t)VERSION_MAJOR << 24;
      value |= (uint32_t)VERSION_MINOR << 16;
      while (!smartport.packetReady()) {
        smartport.processTelemetry();
      }
      smartport.addPacket(0x5001, value);

      value = config.refreshRpm;
      value |= config.refreshVolt << 4;
      value |= (uint32_t)config.refreshCurr << 8;
      value |= (uint32_t)config.refreshTemp << 12;
      value |= (uint32_t)config.queueRpm << 16;
      value |= (uint32_t)config.queueVolt << 20;
      value |= (uint32_t)config.queueCurr << 24;
      value |= (uint32_t)config.queueTemp << 28;
      while (!smartport.packetReady()) {
        smartport.processTelemetry();
      }
      smartport.addPacket(0x5002, value);
    }
    if (dataId == 0x5011) {
      config.protocol = BM_PROTOCOL(value);
      config.voltage1 = BM_VOLTAGE1(value);
      config.voltage2 = BM_VOLTAGE2(value);
      config.current = BM_CURRENT(value);
      config.ntc1 = BM_NTC1(value);
      config.ntc2 = BM_NTC2(value);
      config.pwmOut = BM_PWM(value);
      config.queuePwm = BM_QUEUE_PWM(value);
    }
    if (dataId == 0x5012) {
      config.refreshRpm = BM_REFRESH_RPM(value);
      config.refreshVolt = BM_REFRESH_VOLT(value);
      config.refreshCurr = BM_REFRESH_CURR(value);
      config.refreshTemp = BM_REFRESH_TEMP(value);
      config.queueRpm = BM_QUEUE_RPM(value);
      config.queueVolt = BM_QUEUE_VOLT(value);
      config.queueCurr = BM_QUEUE_CURR(value);
      config.queueTemp = BM_QUEUE_TEMP(value);
      writeConfig();
      initConfig();
      while (!smartport.packetReady()) {
        smartport.processTelemetry();
      }
      smartport.addPacket(0x5020, 0);
    }
  }

#ifdef DEBUG_TELEMETRY2
  Serial.print("Type: ");
  Serial.print(type);
  Serial.print(" DataId: ");
  Serial.print(dataId);
  Serial.print(" Value: ");
  Serial.println(value);
#endif
#ifdef DEBUG_PLOTTER
  _serial.println(DEBUG_PLOTTER);
#endif
}
