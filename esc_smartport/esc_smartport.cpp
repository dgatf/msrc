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
  escSerial.print("Battery: ");
  escSerial.println(config.battery);
  escSerial.print("NTC1: ");
  escSerial.println(config.ntc1);
  escSerial.print("NTC2: ");
  escSerial.println(config.ntc2);
  escSerial.print("Pwm out: ");
  escSerial.println(config.pwmOut);

#endif
}

void writeConfig() {
  EEPROM.put(0, (uint32_t)0x64616E69);
  EEPROM.put(4, config);
#ifdef DEBUG
  escSerial.println("Write config");
  escSerial.print("Protocol: ");
  escSerial.println(config.protocol);
  escSerial.print("Battery: ");
  escSerial.println(config.battery);
  escSerial.print("NTC1: ");
  escSerial.println(config.ntc1);
  escSerial.print("NTC2: ");
  escSerial.println(config.ntc2);
  escSerial.print("Pwm out: ");
  escSerial.println(config.pwmOut);
#endif
}

void initConfig() {

  esc.setPwmOut(config.pwmOut);
  esc.setProtocol(config.protocol);

  smartport.deleteElements();

  switch (config.protocol) {
  case PROTOCOL_HW_V3:
  case PROTOCOL_PWM:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, REFRESH_RPM);
    break;
  case PROTOCOL_HW_V4:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, REFRESH_RPM);
    telemetry.escPowerP =
        smartport.addElement(ESC_POWER_FIRST_ID, REFRESH_VOLT);
    telemetry.temp1P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID, REFRESH_TEMP);
    telemetry.temp2P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID + 1, REFRESH_TEMP);
    break;
  }

  if (config.battery == true) {
    telemetry.voltageAnalogP = smartport.addElement(A3_FIRST_ID, REFRESH_VOLT);
  }
  if (config.ntc1 == true) {
    telemetry.ntc1P = smartport.addElement(T1_FIRST_ID, REFRESH_TEMP);
  }
  if (config.ntc2 == true) {
    telemetry.ntc2P = smartport.addElement(T1_FIRST_ID + 1, REFRESH_TEMP);
  }
}

float readVoltageAnalog() {
  const float analogToVolt = (float)BOARD_VCC / 1024;
  uint16_t value = analogRead(PIN_BATT);
  return value * analogToVolt;
}

float readNtc(uint8_t pin) {
  const float analogToVolt = (float)BOARD_VCC / 1024;
  float volt = analogRead(pin) * analogToVolt;
  float ntcR_Rref = (volt * NTC_R1 / (BOARD_VCC - volt)) / NTC_R_REF;
  /*return
      1 / (NTC_A1 + NTC_B1 * log(ntcR_Rref) + NTC_C1 * pow(log(ntcR_Rref), 2) +
           NTC_D1 * pow(log(ntcR_Rref), 3)) -
      273.15;*/
  return 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
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
  readConfig();
  initConfig();
}

void loop() {
  switch (config.protocol) {
  case PROTOCOL_HW_V3:
    if (esc.read()) {
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(esc.getRpm(), 0);
    }
    break;
  case PROTOCOL_HW_V4:
    if (esc.read()) {
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(esc.getRpm(), 0);
      *telemetry.escPowerP = smartport.formatEscPower(esc.getVolt(), 0);
      *telemetry.temp1P =
          smartport.formatData(ESC_TEMPERATURE_FIRST_ID, esc.getTemp1());
      *telemetry.temp2P =
          smartport.formatData(ESC_TEMPERATURE_FIRST_ID + 1, esc.getTemp2());
    }
    break;
  case PROTOCOL_PWM:
    esc.read();
    *telemetry.escRpmConsP = smartport.formatEscRpmCons(esc.getRpm(), 0);
    break;
  }

  if (config.battery == true) {
    *telemetry.voltageAnalogP =
        smartport.formatData(A3_FIRST_ID, readVoltageAnalog());
  }
  if (config.ntc1 == true) {
    *telemetry.ntc1P = smartport.formatData(T1_FIRST_ID, readNtc(PIN_NTC1));
  }
  if (config.ntc2 == true) {
    *telemetry.ntc2P = smartport.formatData(T1_FIRST_ID + 1, readNtc(PIN_NTC2));
  }

  uint16_t dataId;
  uint32_t value;
  uint8_t type = smartport.processTelemetry(dataId, value);
#ifdef DEBUG
  Serial.print("Type: ");
  Serial.print(type);
  Serial.print(" DataId: ");
  Serial.print(dataId);
  Serial.print(" Value: ");
  Serial.println(value);
#endif
  if (type == PACKET_RECEIVED) {
    if (dataId == 0x5000) {
      uint32_t value = 0;
      value = config.protocol;
      value |= config.battery << 2;
      value |= config.pwmOut << 3;
      value |= config.ntc1 << 4;
      value |= config.ntc2 << 5;
      value |= (uint32_t)VERSION_MAJOR << 24;
      value |= (uint32_t)VERSION_MINOR << 16;
      smartport.addPacket(0x5001, value);
    }
    if (dataId == 0x5002) {
      config.protocol = value & BITMASK_PROTOCOL;
      config.battery = value & BITMASK_BATTERY;
      config.pwmOut = value & BITMASK_PWM;
      config.ntc1 = value & BITMASK_NTC1;
      config.ntc2 = value & BITMASK_NTC2;
      writeConfig();
      initConfig();
      smartport.addPacket(0x5003, 0);
    }
  }
}
