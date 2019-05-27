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
  escSerial.print("V major: ");
  escSerial.println(config.versionMajor);
  escSerial.print("V minor: ");
  escSerial.println(config.versionMinor);
  escSerial.print("Protocol: ");
  escSerial.println(config.protocol);
  escSerial.print("Battery: ");
  escSerial.println(config.battery);
  escSerial.print("Pwm out: ");
  escSerial.println(config.pwmOut);

#endif
}

void writeConfig() {
  EEPROM.put(0, (uint32_t)0x64616E69);
  EEPROM.put(4, config);
#ifdef DEBUG
  escSerial.println("Write config");
  escSerial.print("V major: ");
  escSerial.println(config.versionMajor);
  escSerial.print("V minor: ");
  escSerial.println(config.versionMinor);
  escSerial.print("Protocol: ");
  escSerial.println(config.protocol);
  escSerial.print("Battery: ");
  escSerial.println(config.battery);
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
    telemetry.voltageAnalogP =
        smartport.addElement(A3_FIRST_ID, (uint16_t)REFRESH_VOLT);
  }
}

float readVoltageAnalog() {
  const float analogToVolt = (float)5 / 1024;
  int value = analogRead(PIN_BATT);
  return value * analogToVolt;
}

void setup() {
  escSerial.begin(19200);
  while (!escSerial) {
  }
  escSerial.setTimeout(ESCSERIAL_TIMEOUT);
  smartportSerial.begin(57600);
#ifdef DEBUG
  escSerial.println("DEBUG");
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
  uint16_t dataId;
  uint32_t value;
  uint8_t type = smartport.processTelemetry(dataId, value);
  if (type == PACKET_RECEIVED) {
    if (dataId == 0x5000) {
      uint16_t value = 0;
      value = config.protocol;
      value |= config.battery << 2;
      value |= config.pwmOut << 3;
      value |= config.versionMajor << 4;
      value |= config.versionMinor << 8;
      smartport.addPacket(0x5001, value);
    }
    if (dataId == 0x5002) {
      config.protocol = value & BITMASK_PROTOCOL;
      config.battery = value & BITMASK_BATTERY;
      config.pwmOut = value & BITMASK_PWM;
      writeConfig();
      initConfig();
      smartport.addPacket(0x5003, 0);
    }
  }
}
