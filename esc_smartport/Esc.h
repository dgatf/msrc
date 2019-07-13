#ifndef ESC_H
#define ESC_H

// ESC protocols

#define PROTOCOL_HW_V3 0
#define PROTOCOL_HW_V4 1
#define PROTOCOL_PWM 2

// Pins

#define PIN_PWM_IN 8 // TIMER1 CAPT PIN8

#define COMP_TO_MICROS ((float)8000000UL/F_CPU)

//#define DEBUG_ESC

#include <Arduino.h>

// Pwm in: TIMER 1 CAPT, PIN 8
extern volatile uint16_t pwmInLenght;
extern volatile uint32_t tsPwmIn;

class Esc {

private:
  uint8_t _protocol;
  bool _pwmOut;
  float rpm;
  float voltage;
  float temp1;
  float temp2;
  float current;
  Stream& _serial;

  bool readHWV3();
  bool readHWV4();
  void readPWM();

public:
  Esc(Stream& serial);
  void setProtocol(uint8_t protocol);
  bool read();
  float getRpm();
  float getVolt();
  float getTemp1();
  float getTemp2();
  float getCurrent();
};

#endif
