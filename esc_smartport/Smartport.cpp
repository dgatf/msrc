#include "Smartport.h"

Smartport::Smartport(Stream &serial) : _serial(serial), elementP(), packetP() {
  pinMode(LED_SMARTPORT, OUTPUT);
}

void Smartport::sendByte(uint8_t c, uint16_t *crcp) {
  uint16_t crc = *crcp;
  if (c == 0x7D || c == 0x7E) {
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
    c ^= 0x20;
    _serial.write(0x7D);
    _serial.write(c);
    return;
  }
  _serial.write(c);
  if (crcp == NULL)
    return;
  crc += c;
  crc += crc >> 8;
  crc &= 0x00FF;
  *crcp = crc;
}

void Smartport::sendData(uint16_t dataId, int32_t val) {
  digitalWrite(LED_SMARTPORT, HIGH);
  uint16_t crc = 0;
  uint8_t *u8p;
  // type
  sendByte(0x10, &crc);
  // dataId
  u8p = (uint8_t *)&dataId;
  sendByte(u8p[0], &crc);
  sendByte(u8p[1], &crc);
  // val
  u8p = (uint8_t *)&val;
  sendByte(u8p[0], &crc);
  sendByte(u8p[1], &crc);
  sendByte(u8p[2], &crc);
  sendByte(u8p[3], &crc);
  // crc
  sendByte(0xFF - (uint8_t)crc, NULL);
  digitalWrite(LED_SMARTPORT, LOW);
}

void Smartport::sendVoid() {
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0x00);
  _serial.write((uint8_t)0xFF);
}

/*uint8_t Smartport::readPacket2(uint8_t data[]) {
  uint8_t cont = 0;
  if (_serial.available()) {
    uint16_t tsRead = millis();
    uint16_t crc = 0;
    while ((uint16_t)millis() - tsRead < SMARTPORT_TIMEOUT) {
      if (_serial.available()) {
        data[cont] = _serial.read();
        if (data[cont] == 0x7D) {
          data[cont] = _serial.read() ^ 0x20;
        }
        if (data[cont] == 0x7E) {
          cont = 0;
          crc = 0;
        }
        if (cont > 1 && cont < 9) {
          crc += data[cont];
          crc += crc >> 8;
          crc &= 0x00FF;
        }
        cont++;
      }
    }
    if (cont == 10) {
      crc = 0xFF - (uint8_t)crc;
      if (crc != data[9])
        return PACKET_TYPE_NONE;
      else
        return PACKET_TYPE_PACKET;
    }
    if (cont == 2)
      return PACKET_TYPE_POLL;
  }
  return PACKET_TYPE_NONE;
}*/

uint8_t Smartport::readPacket(uint8_t data[]) {
  uint8_t cont = 0;
  if (_serial.available()) {
    uint16_t tsRead = millis();
    uint16_t crc = 0;
    while ((uint16_t)millis() - tsRead < SMARTPORT_TIMEOUT) {
      if (_serial.available()) {
        data[cont] = _serial.read();
        if (data[cont] == 0x7D) {
          data[cont] = _serial.read() ^ 0x20;
        }
        cont++;
      }
    }
    if (data[0] != 0x7E)
      return PACKET_TYPE_NONE;
    if (cont == 10) {
      for (uint8_t i = 2; i < 9; i++) {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00FF;
      }
      crc = 0xFF - (uint8_t)crc;
      if (crc != data[9])
        return PACKET_TYPE_NONE;
      else
        return PACKET_TYPE_PACKET;
    }
    if (cont == 2)
      return PACKET_TYPE_POLL;
  }
  return PACKET_TYPE_NONE;
}

uint8_t Smartport::available() { return _serial.available(); }

uint32_t Smartport::formatData(uint16_t dataId, float value) {

  if ((dataId >= GPS_SPEED_FIRST_ID && dataId <= GPS_SPEED_LAST_ID) ||
      (dataId >= RBOX_BATT1_FIRST_ID && dataId <= RBOX_BATT2_FIRST_ID))
    return value * 1000;

  if ((dataId >= ALT_FIRST_ID && dataId <= VARIO_LAST_ID) ||
      (dataId >= VFAS_FIRST_ID && dataId <= VFAS_LAST_ID) ||
      (dataId >= ACCX_FIRST_ID && dataId <= GPS_ALT_LAST_ID) ||
      (dataId >= GPS_COURS_FIRST_ID && dataId <= GPS_COURS_LAST_ID) ||
      (dataId >= A3_FIRST_ID && dataId <= A4_LAST_ID))
    return value * 100;

  if ((dataId >= CURR_FIRST_ID && dataId <= CURR_LAST_ID) ||
      (dataId >= AIR_SPEED_FIRST_ID && dataId <= AIR_SPEED_LAST_ID) ||
      dataId == A1_ID || dataId == A2_ID || dataId == RXBT_ID)
    return value * 10;

  return value;
}

uint32_t Smartport::formatEscPower(float volt, float curr) {
  return (uint32_t)(curr * 100) << 16 | (uint16_t)(volt * 100);
}

uint32_t Smartport::formatEscRpmCons(float rpm, float cons) {
  return (uint32_t)cons << 16 | (uint16_t)(rpm / 100);
}

uint32_t Smartport::formatCell(uint8_t cellId, float val) {
  val *= 500;
  return (uint8_t)val << 8 | cellId;
}

float *Smartport::addElement(uint16_t dataId, uint16_t refresh) {
  Element *newElementP = (Element *)malloc(sizeof(Element));
  static Element *prevElementP;

  if (elementP == NULL) {
    elementP = newElementP;
    prevElementP = elementP;
  }

  newElementP->nextP = elementP;
  prevElementP->nextP = newElementP;
  prevElementP = newElementP;

  newElementP->ts = 0;
  newElementP->dataId = dataId;
  newElementP->refresh = refresh / 100;
  newElementP->value = 0;

  return &newElementP->value;
}

bool Smartport::addPacket(uint16_t dataId, uint32_t value) {
  if (packetP == NULL) {
    packetP = (Packet *)malloc(sizeof(Packet));
    packetP->dataId = dataId;
    packetP->value = value;
    return true;
  }
  return false;
}

void Smartport::deleteElements() {
  if (elementP != NULL) {
    Element *delelemP = elementP;
    Element *nextelemP;
    do {
      nextelemP = delelemP->nextP;
      free(delelemP);
      delelemP = nextelemP;
    } while (nextelemP != elementP);
    elementP = NULL;
  }
}

uint8_t Smartport::processTelemetry(uint16_t &dataId, uint32_t &value) {
  if (available()) {
    uint8_t data[64];
    uint8_t type;
    type = readPacket(data);
    if (type == PACKET_TYPE_POLL && data[1] == SMARTPORT_SENSOR) {
      if (packetP != NULL) {
        sendData(packetP->dataId, packetP->value);
        dataId = packetP->dataId;
        value = packetP->value;
        free(packetP);
        packetP = NULL;
        return PACKET_SENT;
      }
      if (elementP != NULL) {
          if ((uint16_t)millis() - elementP->ts >=
          (uint16_t)elementP->refresh * 100) {
          sendData(elementP->dataId, elementP->value);
          elementP->ts = millis();
          elementP = elementP->nextP;
          dataId = elementP->dataId;
          value = elementP->value;
          return PACKET_SENT_TELEMETRY;
        } else {
          dataId = 0;
          value = 0;
          sendVoid();
          elementP = elementP->nextP;
          return PACKET_SENT_VOID;
        }
      }
    } else if (type == PACKET_TYPE_PACKET && data[1] == SMARTPORT_SENSOR_TX) {
      dataId = (uint16_t)data[4] << 8 | data[3];
      value = (uint32_t)data[8] << 24 | (uint32_t)data[7] << 16 | (uint16_t)data[6] << 8 |
              data[5];
      return PACKET_RECEIVED;
    }
  }
  return PACKET_NONE;
}

uint8_t Smartport::processTelemetry() {
  uint16_t dataId;
  uint32_t value;
  return processTelemetry(dataId, value);
}
