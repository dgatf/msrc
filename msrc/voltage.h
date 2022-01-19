#ifndef VOLTAGE_H
#define VOLTAGE_H

#if F_CPU == 16000000UL
#define BOARD_VCC 5
#else
#define BOARD_VCC 3.3
#endif

#if defined(__AVR_ATmega328P__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega2560__)  || defined(__AVR_ATmega32U4__) 
#define ADC_RESOLUTION 1024.0
#else
#define ADC_RESOLUTION 4096.0
#endif

#include <Arduino.h>
#include "device.h"

class Voltage : public AbstractDevice
{
protected:
    uint8_t pin_;
    float value_ = 0;
    uint8_t alpha_;
    float multiplier_ = 1;
    float readVoltage();

public:
    Voltage(uint8_t pin, uint8_t alpha, float multiplier);
    Voltage(uint8_t pin, uint8_t alpha);
    void update();
    float *valueP();
};

#endif