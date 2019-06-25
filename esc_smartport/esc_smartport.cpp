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
#endif
}

void initConfig() {

  esc.setPwmOut(config.pwmOut);
  esc.setProtocol(config.protocol);

  smartport.deleteElements();

  telemetry.rpmQ.del(QUEUE_RPM);
  telemetry.voltageQ.del(QUEUE_VOLT);
  telemetry.temp1Q.del(QUEUE_TEMP);
  telemetry.temp2Q.del(QUEUE_TEMP);
  telemetry.voltageAnalog1Q.del(QUEUE_VOLT);
  telemetry.voltageAnalog2Q.del(QUEUE_VOLT);
  telemetry.currentAnalogQ.del(QUEUE_CURR);
  telemetry.ntc1Q.del(QUEUE_TEMP);
  telemetry.ntc2Q.del(QUEUE_TEMP);

  switch (config.protocol) {
  case PROTOCOL_HW_V3:
  case PROTOCOL_PWM:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, REFRESH_RPM);
    telemetry.rpmQ.init(0, QUEUE_RPM);
    break;
  case PROTOCOL_HW_V4:
    telemetry.escRpmConsP =
        smartport.addElement(ESC_RPM_CONS_FIRST_ID, REFRESH_RPM);
    telemetry.rpmQ.init(0, QUEUE_RPM);
    telemetry.escPowerP =
        smartport.addElement(ESC_POWER_FIRST_ID, REFRESH_VOLT);
    telemetry.voltageQ.init(0, QUEUE_VOLT);
    telemetry.temp1P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID, REFRESH_TEMP);
    telemetry.temp1Q.init(0, QUEUE_TEMP);
    telemetry.temp2P =
        smartport.addElement(ESC_TEMPERATURE_FIRST_ID + 1, REFRESH_TEMP);
    telemetry.temp2Q.init(0, QUEUE_TEMP);
    break;
  }

  if (config.voltage1 == true) {
    telemetry.voltageAnalog1P = smartport.addElement(A3_FIRST_ID, REFRESH_VOLT);
    telemetry.voltageAnalog1Q.init(0, QUEUE_VOLT);
  }
  if (config.voltage2 == true) {
    telemetry.voltageAnalog2P = smartport.addElement(A4_FIRST_ID, REFRESH_VOLT);
    telemetry.voltageAnalog2Q.init(0, QUEUE_VOLT);
  }
  if (config.current == true) {
    telemetry.currentAnalogP =
        smartport.addElement(CURR_FIRST_ID, REFRESH_CURR);
    telemetry.currentAnalogQ.init(0, QUEUE_CURR);
  }
  if (config.ntc1 == true) {
    telemetry.ntc1P = smartport.addElement(T1_FIRST_ID, REFRESH_TEMP);
    telemetry.ntc1Q.init(0, QUEUE_TEMP);
  }
  if (config.ntc2 == true) {
    telemetry.ntc2P = smartport.addElement(T2_FIRST_ID, REFRESH_TEMP);
    telemetry.ntc2Q.init(0, QUEUE_TEMP);
  }
}

float readVoltageAnalog(uint8_t pin) {
  const float analogToVolt = (float)BOARD_VCC / 1024;
  uint16_t value = analogRead(pin);
  return value * analogToVolt;
}

float readNtc(uint8_t pin) {
  float volt = readVoltageAnalog(pin);
  float ntcR_Rref = (volt * NTC_R1 / (BOARD_VCC - volt)) / NTC_R_REF;
  if (ntcR_Rref < 1)
    return 0;
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
  
  float valueTelemetry;
  switch (config.protocol) {
  case PROTOCOL_HW_V3:
    esc.read();
     valueTelemetry = esc.getRpm() / QUEUE_RPM;
    telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
    telemetry.rpmQ.enqueue(valueTelemetry);
    *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);
    break;
  case PROTOCOL_HW_V4:
    if (esc.read()) {

      valueTelemetry = esc.getRpm() / QUEUE_RPM;
      telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
      telemetry.rpmQ.enqueue(valueTelemetry);
      *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);

      valueTelemetry = esc.getVolt() / QUEUE_VOLT;
      telemetry.voltageAvg += valueTelemetry - telemetry.voltageQ.dequeue();
      telemetry.voltageQ.enqueue(valueTelemetry);
      *telemetry.escRpmConsP =
          smartport.formatEscPower(telemetry.voltageAvg, 0);

      valueTelemetry = esc.getTemp1() / QUEUE_TEMP;
      telemetry.temp1Avg += valueTelemetry - telemetry.temp1Q.dequeue();
      telemetry.temp1Q.enqueue(valueTelemetry);
      *telemetry.escRpmConsP =
          smartport.formatData(ESC_TEMPERATURE_FIRST_ID, telemetry.temp1Avg);

      valueTelemetry = esc.getTemp2() / QUEUE_TEMP;
      telemetry.temp2Avg += valueTelemetry - telemetry.temp2Q.dequeue();
      telemetry.temp2Q.enqueue(valueTelemetry);
      *telemetry.escRpmConsP = smartport.formatData(
          ESC_TEMPERATURE_FIRST_ID + 1, telemetry.temp1Avg);
    }
    break;
  case PROTOCOL_PWM:
    esc.read();
    valueTelemetry = esc.getRpm() / QUEUE_RPM;
    telemetry.rpmAvg += valueTelemetry - telemetry.rpmQ.dequeue();
    telemetry.rpmQ.enqueue(valueTelemetry);
    *telemetry.escRpmConsP = smartport.formatEscRpmCons(telemetry.rpmAvg, 0);
    break;
  }

  if (config.voltage1 == true) {
    valueTelemetry = readVoltageAnalog(PIN_VOLTAGE1) / QUEUE_VOLT;
    telemetry.voltageAnalog1Avg += valueTelemetry - telemetry.voltageAnalog1Q.dequeue();
    telemetry.voltageAnalog1Q.enqueue(valueTelemetry);
    *telemetry.voltageAnalog1P =
        smartport.formatData(A3_FIRST_ID, telemetry.voltageAnalog1Avg);
  }
  if (config.voltage2 == true) {
    valueTelemetry = readVoltageAnalog(PIN_VOLTAGE2) / QUEUE_VOLT;
    telemetry.voltageAnalog2Avg += valueTelemetry - telemetry.voltageAnalog2Q.dequeue();
    telemetry.voltageAnalog2Q.enqueue(valueTelemetry);
    *telemetry.voltageAnalog2P =
        smartport.formatData(A4_FIRST_ID, telemetry.voltageAnalog2Avg);
  }
  if (config.current == true) {
    valueTelemetry = readVoltageAnalog(PIN_CURRENT) / QUEUE_CURR;
    telemetry.currentAnalogAvg += valueTelemetry - telemetry.currentAnalogQ.dequeue();
    telemetry.currentAnalogQ.enqueue(valueTelemetry);
    *telemetry.currentAnalogP =
        smartport.formatData(CURR_FIRST_ID, telemetry.currentAnalogAvg);
  }
  if (config.ntc1 == true) {
    valueTelemetry = readNtc(PIN_NTC1) / QUEUE_TEMP;
    telemetry.ntc1Avg += valueTelemetry - telemetry.ntc1Q.dequeue();
    telemetry.ntc1Q.enqueue(valueTelemetry);
    *telemetry.ntc1P = smartport.formatData(T1_FIRST_ID, telemetry.ntc1Avg);
  }
  if (config.ntc2 == true) {
    float valueTelemetry = readNtc(PIN_NTC2) / QUEUE_TEMP;
    telemetry.ntc2Avg += valueTelemetry - telemetry.ntc2Q.dequeue();
    telemetry.ntc2Q.enqueue(valueTelemetry);
    *telemetry.ntc2P = smartport.formatData(T2_FIRST_ID, telemetry.ntc2Avg);
  }

  uint16_t dataId;
  uint32_t value;
  uint8_t type = smartport.processTelemetry(dataId, value);
#ifdef DEBUG2
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
      value |= config.voltage1 << 2;
      value |= config.voltage2 << 3;
      value |= config.current << 4;
      value |= config.ntc1 << 5;
      value |= config.ntc2 << 6;
      value |= config.pwmOut << 7;
      value |= (uint32_t)VERSION_MAJOR << 24;
      value |= (uint32_t)VERSION_MINOR << 16;
      smartport.addPacket(0x5001, value);
    }
    if (dataId == 0x5002) {
      config.protocol = value & BITMASK_PROTOCOL;
      config.voltage1 = value & BITMASK_VOLTAGE1;
      config.voltage2 = value & BITMASK_VOLTAGE2;
      config.current = value & BITMASK_CURRENT;
      config.ntc1 = value & BITMASK_NTC1;
      config.ntc2 = value & BITMASK_NTC2;
      config.pwmOut = value & BITMASK_PWM;

      writeConfig();
      initConfig();
      smartport.addPacket(0x5003, 0);
    }
  }
}
