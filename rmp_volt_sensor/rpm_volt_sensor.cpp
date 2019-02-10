#include "rpm_volt_sensor.h"

void queueInit() {
  for (int8_t i = 0; i < RPM_QUEUE_SIZE; i++)
    queueRpm.enqueue(0);
#ifdef BATT_SENSOR_VOLT
  for (int8_t i = 0; i < VOLT_QUEUE_SIZE; i++)
    queueVolt.enqueue(0);
#endif
}

void sendByte(uint8_t c, uint16_t *crcp) {
  if (c == 0x7D || c == 0x7E) {
    smartportSerial.write(0x7D);
    c ^= 0x20;
  }

  smartportSerial.write(c);

  if (crcp == NULL)
    return;

  uint16_t crc = *crcp;
  crc += c;
  crc += crc >> 8;
  crc &= 0x00FF;
  *crcp = crc;
}

void sendData(uint16_t id, int32_t val) {
  uint16_t crc = 0;
  uint8_t *u8p;
  uint8_t type = 0x10;

  // type
  sendByte(type, &crc);

  // id
  u8p = (uint8_t *)&id;
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
}

uint32_t lipoCell(uint8_t id, float val1, float val2) {
  val1 *= 500;
  val2 *= 500;
  return ((uint32_t)val2 & 0x0fff) << 20 | ((uint32_t)val1 & 0x0fff) << 8 |
         (CELLS & 0x0f) << 4 | (id & 0x0f);
}

uint32_t lipoCell(uint8_t id, float val) {
  val *= 500;
  return (uint32_t)val << 8 | CELLS << 4 | id;
}

void sendVolt(float volt) {
  bool status = false;
  smartportSerial.listen();
  smartportSerial.setTimeout(20);
  while (status == false) {
    if (smartportSerial.available()) {
      if (smartportSerial.read() == 0x7E) {
        while (!smartportSerial.available()) {
        }
        if (smartportSerial.read() == 0x22) {
          sendData(0x210, (uint16_t)(volt * 100));
          status = true;
        }
      }
    }
  }
}

void sendCell(float cell1, float cell2, float cell3) {
  smartportSerial.listen();
  smartportSerial.setTimeout(20);
  uint8_t status = 0;
  while (status < 2) {
    if (smartportSerial.available()) {
      if (smartportSerial.read() == 0x7E) {
        while (!smartportSerial.available())
          ;
        if (smartportSerial.read() == 0xA1) {
          if (status == 0) {
            sendData(0x300, lipoCell(0, cell1, cell2));
            status = 1;
          } else {
            sendData(0x300, lipoCell(2, cell3));
            status = 2;
          }
        }
      }
    }
  }
}

void sendRpm(float rpm) {
  if (rpm != -1) {
    bool status = false;
    smartportSerial.listen();
    smartportSerial.setTimeout(20);
    while (status == false) {
      if (smartportSerial.available()) {
        if (smartportSerial.read() == 0x7E) {
          while (!smartportSerial.available()) {
          }
          if (smartportSerial.read() == 0xE4) {
            sendData(0x500, rpm);
            status = true;
          }
        }
      }
    }
  }
}

float escDigitalRead(SoftwareSerial &escSerial) {
  escSerial.listen();
  delay(20);
  escSerial.setTimeout(10);
  if (escSerial.available() >= 10) {
    escSerial.find(155);
    uint8_t data[10];
    uint8_t cont = escSerial.readBytesUntil(155, data, 10);
    if (cont == 10 and data[4] == 0 and data[6] == 0) {
      uint16_t rpmCycle = data[8] << 8 | data[9];
      return 6e7 / ((float)rpmCycle * POLES);
    } else {
      return -1; // error
    }
  } else {
    return 0;
  }
}

float escPwmRead() {
  uint16_t pwm_value = pulseIn(PIN_ESC, HIGH);
  if (pwm_value != 0) {
    return (float)100000000 / pwm_value;
  } else
    return 0;
}

void readCell(float &cell1, float &cell2, float &cell3) {
  const float analogToVolt = (float)5 / 1024;
  cell1 = analogRead(PIN_CELL1) * analogToVolt;
  cell2 = analogRead(PIN_CELL2) * analogToVolt;
  cell3 = analogRead(PIN_CELL3) * analogToVolt;
}

float readVolt() {
  const float analogToVolt = (float)5 / 1024;
  int value = analogRead(PIN_BATT);
  return value * analogToVolt;
}

void setup() {
#ifdef BATT_SENSOR_CELLS
  pinMode(PIN_CELL1, INPUT);
  pinMode(PIN_CELL2, INPUT);
  pinMode(PIN_CELL3, INPUT);
#endif
#ifdef BATT_SENSOR_VOLT
  pinMode(PIN_BATT, INPUT);
#endif
  smartportSerial.begin(57600);
#ifdef ESC_DIGITAL
  escSerial.begin(19200);
#endif
  queueInit();
}

void loop() {
  float rpm;
#ifdef ESC_DIGITAL
  rpm = escDigitalRead(escSerial);
#else
  rpm = escPwmRead();
#endif
  queueRpm.enqueue(rpm);
  avRpm += rpm - queueRpm.front();
  sendRpm(avRpm / RPM_QUEUE_SIZE);
  queueRpm.dequeue();

#ifdef BATT_SENSOR_CELLS
  float cell1 = 0;
  float cell2 = 0;
  float cell3 = 0;
  readCell(cell1, cell2, cell3);
  sendCell(cell1, cell2, cell3);
#endif
#ifdef BATT_SENSOR_VOLT
  float volt = readVolt();
  queueVolt.enqueue(volt);
  avVolt += volt - queueVolt.front();
  sendVolt(avVolt / VOLT_QUEUE_SIZE);
  queueVolt.dequeue();
  // Serial.println(avVolt / VOLT_QUEUE_SIZE);
#endif
}
