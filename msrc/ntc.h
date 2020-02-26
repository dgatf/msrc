#ifndef NTC_H
#define NTC_H

// Thermistors (NTC 100k, R1 10k)
#define NTC_R_REF 100000UL
#define NTC_R1 10000
#define NTC_BETA 4190
//#define NTC_A1 3.35E-03
//#define NTC_B1 2.46E-04
//#define NTC_C1 3.41E-06
//#define NTC_D1 1.03E-07

#include <Arduino.h>
#include "voltage.h"

class NtcInterface : public VoltageInterface
{
    private:
	public:
        NtcInterface(uint8_t pin, uint8_t alpha);
        float read(uint8_t index);			
};

#endif