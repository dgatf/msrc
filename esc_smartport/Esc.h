#ifndef ESC_H
#define ESC_H

// ESC protocols

#define PROTOCOL_HW_V3 0
#define PROTOCOL_HW_V4 1
#define PROTOCOL_CASTLE 2
#define PROTOCOL_PWM 3

// Pins

#define PIN_RX 2 // only pins 2 or 3. ext int
#define PIN_CASTLE 9 // only pin 9. timer1 pwm
#define PIN_PWM_IN 8 // only pin 8. timer1 capt


#define F_CPU_SCALER (uint8_t)((uint32_t)F_CPU/8000000)
#define COMP_TO_MS (float)8000/F_CPU

//#define DEBUG

#include <Arduino.h>

// Isr telemetry
struct IsrTelemetry {
  uint16_t ms1 = 0;
  uint16_t ms = 0;
  float voltage;
  float rippleVoltage;
  float current;
  float rpm;
  float becVoltage;
  float becCurrent;
  float temperature;
};

// Pwm in: TIMER 1 CAPT, PIN 8
extern volatile uint16_t pwmInLenght;

// Rx in: EXT INT0
extern volatile uint16_t rxLenght;

// Castle telemetry. EXT INT1
extern volatile uint16_t castleInit;
extern volatile uint16_t castleLenght;
extern volatile uint8_t cont;

// TIMER2: disable PCINT for Rx
extern volatile uint8_t contTimer2;
extern volatile uint8_t  _PCMSK0;
extern volatile uint8_t  _PCMSK1;
extern volatile uint8_t  _PCMSK2;

extern volatile IsrTelemetry isrTelemetry;

class Esc {

private:
  uint8_t _protocol;
  bool _pwmOut;
  float rpm;
  float voltage;
  float temp1;
  float temp2;
  float rippleVoltage;
  float current;
  float becVoltage;
  float becCurrent;
  Stream& _serial;

  bool readHWV3();
  bool readHWV4();
  void readPWM();
  bool readCastle();

public:
  Esc(Stream& serial);
  void setProtocol(uint8_t protocol);
  void setPwmOut(uint8_t pwmOut);
  bool read();
  float getRpm();
  float getVolt();
  float getTemp1();
  float getTemp2();
  float getRippleVoltage();
  float getCurrent();
  float getBecVoltage();
  float getBecCurrent();
};

#endif
