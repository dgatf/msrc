#ifndef ESC_H
#define ESC_H

// ESC protocols

#define PROTOCOL_HW_V3 0
#define PROTOCOL_HW_V4 1
#define PROTOCOL_CASTLE 2
#define PROTOCOL_PWM 3

// Pins

#define PIN_PWM_IN_RX 2 // only pins 2 or 3
#define PIN_PWM_OUT 4
#define PIN_CASTLE 3 // only pins 2 or 3

// ISR Timer1

#define TIMER_PWM_OUT 0
#define TIMER_CASTLE 1

// ISR Int0

#define INT0_PWM_IN 0
#define INT0_RX 1

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

// Pwm in INT0
extern volatile uint16_t pwmInInit;
extern volatile uint16_t pwmInLenght;
extern volatile uint8_t isr_int0;

// Pwm out TIMER1
extern volatile uint16_t pwmOutLow;
extern volatile uint16_t pwmOutHigh;
extern volatile bool pwmOutActive;
extern volatile bool pwmOutState;
extern volatile uint8_t isr_timer;

// Castle INT1
extern volatile uint16_t castlelinkInit;
extern volatile uint16_t castlelinkLenght;
extern volatile uint8_t cont;

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
