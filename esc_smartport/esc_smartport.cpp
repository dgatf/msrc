#include "esc_smartport.h"

void readConfig() {
  uint32_t chk;
  EEPROM.get(0, chk);
  if (chk == 0x64616E69) {
    EEPROM.get(4, config);
    if (config.refreshRpm > 16)
      config.refreshRpm = 2;
    if (config.refreshVolt > 16)
      config.refreshVolt = 10;
    if (config.refreshCurr > 16)
      config.refreshCurr = 10;
    if (config.refreshTemp > 16)
      config.refreshTemp = 10;
    if (config.alphaRpm > 16)
      config.alphaRpm = calcAlpha(5);
    if (config.alphaVolt > 16)
      config.alphaVolt = calcAlpha(5);
    if (config.alphaCurr > 16)
      config.alphaCurr = calcAlpha(5);
    if (config.alphaTemp > 16)
      config.alphaTemp = calcAlpha(5);
    if (config.alphaPwm > 16)
      config.alphaPwm = calcAlpha(5);
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
  escSerial.print("alpha RPM: ");
  escSerial.println(config.alphaRpm);
  escSerial.print("alpha Volt: ");
  escSerial.println(config.alphaVolt);
  escSerial.print("alpha Curr: ");
  escSerial.println(config.alphaCurr);
  escSerial.print("alpha Temp: ");
  escSerial.println(config.alphaTemp);
  escSerial.print("alpha PWM: ");
  escSerial.println(config.alphaPwm);
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
  escSerial.print("alpha RPM: ");
  escSerial.println(config.alphaRpm);
  escSerial.print("alpha Volt: ");
  escSerial.println(config.alphaVolt);
  escSerial.print("alpha Curr: ");
  escSerial.println(config.alphaCurr);
  escSerial.print("alpha Temp: ");
  escSerial.println(config.alphaTemp);
  escSerial.print("alpha PWM: ");
  escSerial.println(config.alphaPwm);
#endif
}

void initConfig() {

  esc.setProtocol(config.protocol);
  setPwmOut();

  smartport.deleteElements();

  switch (config.protocol) {
  case PROTOCOL_HW_V3:
  case PROTOCOL_PWM:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, config.refreshRpm * 100);
    break;
  case PROTOCOL_HW_V4:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, config.refreshRpm * 100);
    telemetry.escPowerP =
        smartport.addElement(ESC_POWER_FIRST_ID, config.refreshVolt * 100);
    telemetry.cellP =
        smartport.addElement(VFAS_FIRST_ID, config.refreshVolt * 100);
    telemetry.temp1P = smartport.addElement(ESC_TEMPERATURE_FIRST_ID,
                                            config.refreshTemp * 100);
    telemetry.temp2P = smartport.addElement(ESC_TEMPERATURE_FIRST_ID + 1,
                                            config.refreshTemp * 100);
    break;
  }

  if (config.voltage1 == true) {
    telemetry.voltageAnalog1P =
        smartport.addElement(A3_FIRST_ID, config.refreshVolt * 100);
  }
  if (config.voltage2 == true) {
    telemetry.voltageAnalog2P =
        smartport.addElement(A4_FIRST_ID, config.refreshVolt * 100);
  }
  if (config.current == true) {
    telemetry.currentAnalogP =
        smartport.addElement(CURR_FIRST_ID, config.refreshCurr * 100);
  }
  if (config.ntc1 == true) {
    telemetry.ntc1P =
        smartport.addElement(T1_FIRST_ID, config.refreshTemp * 100);
  }
  if (config.ntc2 == true) {
    telemetry.ntc2P =
        smartport.addElement(T2_FIRST_ID, config.refreshTemp * 100);
  }
}

float calcAlpha(uint8_t elements) {
    return 2 / (elements + 1);
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

uint8_t setCellCount(float voltage) {
  if (voltage > 42) return 12;
  if (voltage > 33.6) return 10;
  if (voltage > 29.4) return 8;
  if (voltage > 25.2) return 7;
  if (voltage > 21) return 6;
  if (voltage > 16.8) return 5;
  if (voltage > 12.6) return 4;
  if (voltage > 8.4) return 3;
  if (voltage > 4.2) return 2;
  return 1;
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
  if (temp < 0)
    return 0;
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
  escSerial.print(VERSION_MINOR);
  escSerial.print(".");
  escSerial.println(VERSION_PATCH);
#endif

#ifdef CONFIG_LUA
  readConfig();
#endif

  initConfig();
}

void loop() {

  bool statusChange = esc.read();

  switch (config.protocol) {

  case PROTOCOL_HW_V3:
    if (statusChange || esc.getRpm() == 0) {
      telemetry.rpm += telemetry.rpm + config.alphaRpm * (esc.getRpm() - telemetry.rpm);
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpm, 0);
    }
    break;

  case PROTOCOL_HW_V4:
    if (statusChange) {
      if (telemetry.cellCount == 0 && millis() > 2000) telemetry.cellCount = setCellCount(telemetry.voltage);

      telemetry.rpm += telemetry.rpm + config.alphaRpm * (esc.getRpm() - telemetry.rpm);
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpm, 0);

      telemetry.voltage += telemetry.voltage + config.alphaVolt * (esc.getVolt() - telemetry.voltage);
      *telemetry.cellP = smartport.formatData(VFAS_FIRST_ID, telemetry.voltage / telemetry.cellCount);

      telemetry.current += telemetry.current + config.alphaCurr * (esc.getCurrent() - telemetry.current);
      *telemetry.escPowerP = smartport.formatEscPower(telemetry.voltage, telemetry.current);

      telemetry.temp1 += telemetry.temp1 + config.alphaTemp * (esc.getTemp1() - telemetry.temp1);
      *telemetry.temp1P =
          smartport.formatData(ESC_TEMPERATURE_FIRST_ID, telemetry.temp1);

      telemetry.temp2 += telemetry.temp2 + config.alphaTemp * (esc.getTemp2() - telemetry.temp2);
      *telemetry.temp2P = smartport.formatData(ESC_TEMPERATURE_FIRST_ID + 1,
                                               telemetry.temp2);
    }
    break;

  case PROTOCOL_PWM:
    telemetry.rpm += telemetry.rpm + config.alphaRpm * (esc.getRpm() - telemetry.rpm);
    *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpm, 0);
    break;
  }

  if (config.pwmOut == true && config.protocol != PROTOCOL_PWM &&
      statusChange) {
    telemetry.pwm += telemetry.pwm + config.alphaPwm * (esc.getRpm() - telemetry.pwm);
    noInterrupts();
    if (esc.getRpm() >= 2000) {
#if MODE_PWM_OUT == ICR
      // ICR
      TCCR1A |= _BV(COM1A1);
      ICR1 = (7.5 * (uint32_t)F_CPU / telemetry.pwm) - 1;
      OCR1A = DUTY * ICR1;
#else
      // OCR
      TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
      OCR1A = (7.5 * (uint32_t)F_CPU / telemetry.pwm) - 1;
      OCR1B = DUTY * OCR1A;
#endif
    } else {
      TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
    }
    interrupts();
  }

  if (config.voltage1 == true) {
    telemetry.voltageAnalog1 += telemetry.voltageAnalog1 + config.alphaVolt * (readVoltageAnalog(PIN_VOLTAGE1) - telemetry.voltageAnalog1);
    *telemetry.voltageAnalog1P =
        smartport.formatData(A3_FIRST_ID, telemetry.voltageAnalog1);
  }
  if (config.voltage2 == true) {
    telemetry.voltageAnalog2 += telemetry.voltageAnalog2 + config.alphaVolt * (readVoltageAnalog(PIN_VOLTAGE2) - telemetry.voltageAnalog2);
    *telemetry.voltageAnalog2P =
        smartport.formatData(A4_FIRST_ID, telemetry.voltageAnalog2);
  }
  if (config.current == true) {
    telemetry.currentAnalog += telemetry.currentAnalog + config.alphaCurr * (readVoltageAnalog(PIN_CURRENT) - telemetry.currentAnalog);
    *telemetry.currentAnalogP =
        smartport.formatData(CURR_FIRST_ID, telemetry.currentAnalog);
  }
  if (config.ntc1 == true) {
    telemetry.ntc1 += telemetry.ntc1 + config.alphaTemp * (readNtc(PIN_NTC1) - telemetry.ntc1);
    *telemetry.ntc1P = smartport.formatData(T1_FIRST_ID, telemetry.ntc1);
  }
  if (config.ntc2 == true) {
    telemetry.ntc2 += telemetry.ntc2 + config.alphaTemp * (readNtc(PIN_NTC2) - telemetry.ntc2);
    *telemetry.ntc2P = smartport.formatData(T2_FIRST_ID, telemetry.ntc2);
  }

  uint8_t frameId;
  uint16_t dataId;
  uint32_t value;
  uint8_t type = smartport.processSmartport(frameId, dataId, value);

  if (type == PACKET_RECEIVED) {
#ifdef DEBUG
    Serial.print("Type: ");
    Serial.print(type, HEX);
    Serial.print(" FrameId: ");
    Serial.print(frameId, HEX);
    Serial.print(" DataId: ");
    Serial.print(dataId, HEX);
    Serial.print(" Value: ");
    Serial.println(value);
#endif
    if (frameId == 0x21 && dataId == 0xFFFF && value == 0x80) {
      smartport.maintenanceMode(true);
    }
    if (frameId == 0x20 && dataId == 0xFFFF && value == 0x80) {
      smartport.maintenanceMode(false);
    }
    if (smartport.maintenanceMode()) {
      if (dataId == 0x5000) {
        uint32_t value = 0;

        // packet 1
        value = VERSION_PATCH;
        value |= (uint32_t)VERSION_MINOR << 8;
        value |= (uint32_t)VERSION_MAJOR << 16;
        while (!smartport.packetReady()) {
          smartport.processSmartport();
        }
        smartport.addPacket(0x5001, value);
#ifdef DEBUG
        Serial.print("Sent 0x5001: ");
        Serial.println(value);
#endif

        // packet 2
        value = config.protocol;
        value |= config.voltage1 << 2;
        value |= config.voltage2 << 3;
        value |= config.current << 4;
        value |= config.ntc1 << 5;
        value |= config.ntc2 << 6;
        value |= config.pwmOut << 7;
        value |= (uint32_t)config.refreshRpm << 8;
        value |= (uint32_t)config.refreshVolt << 12;
        value |= (uint32_t)config.refreshCurr << 16;
        value |= (uint32_t)config.refreshTemp << 20;
        while (!smartport.packetReady()) {
          smartport.processSmartport();
        }
        smartport.addPacket(0x5002, value);
#ifdef DEBUG
        Serial.print("Sent 0x5002: ");
        Serial.println(value);
#endif

        // packet 3
        value = (uint32_t)(2 / config.alphaRpm - 1);
        value |= (uint32_t)(2 / config.alphaVolt -1) << 4;
        value |= (uint32_t)(2 / config.alphaCurr - 1) << 8;
        value |= (uint32_t)(2 / config.alphaTemp - 1) << 12;
        value |= (uint32_t)(2 / config.alphaPwm - 1) << 16;
        while (!smartport.packetReady()) {
          smartport.processSmartport();
        }
        smartport.addPacket(0x5003, value);
#ifdef DEBUG
        Serial.print("Sent 0x5003: ");
        Serial.println(value);
#endif
      }
      if (dataId == 0x5011) {
        config.protocol = BM_PROTOCOL(value);
        config.voltage1 = BM_VOLTAGE1(value);
        config.voltage2 = BM_VOLTAGE2(value);
        config.current = BM_CURRENT(value);
        config.ntc1 = BM_NTC1(value);
        config.ntc2 = BM_NTC2(value);
        config.pwmOut = BM_PWM(value);
        config.refreshRpm = BM_REFRESH_RPM(value);
        config.refreshVolt = BM_REFRESH_VOLT(value);
        config.refreshCurr = BM_REFRESH_CURR(value);
        config.refreshTemp = BM_REFRESH_TEMP(value);
        while (!smartport.packetReady()) {
          smartport.processSmartport();
        }
        smartport.addPacket(0x5020, 0);
      }
      if (dataId == 0x5012) {
        config.alphaRpm = calcAlpha(BM_AVG_ELEM_RPM(value));
        config.alphaVolt = calcAlpha(BM_AVG_ELEM_VOLT(value));
        config.alphaCurr = calcAlpha(BM_AVG_ELEM_CURR(value));
        config.alphaTemp = calcAlpha(BM_AVG_ELEM_TEMP(value));
        config.alphaPwm = calcAlpha(BM_AVG_ELEM_PWM(value));
        writeConfig();
        initConfig();
        while (!smartport.packetReady()) {
          smartport.processSmartport();
        }
        smartport.addPacket(0x5021, 0);
      }
    }
  }

#ifdef DEBUG_TELEMETRY
  Serial.print("Type: ");
  Serial.print(type);
  Serial.print(" DataId: ");
  Serial.print(dataId);
  Serial.print(" Value: ");
  Serial.println(value);
#endif
#ifdef DEBUG_PLOTTER
  Serial.println(DEBUG_PLOTTER);
#endif
}
