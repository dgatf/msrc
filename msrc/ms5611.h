#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include "device.h"
#include "i2c.h"

#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_CONV_D1            0x40
#define MS5611_CMD_CONV_D2            0x50
#define MS5611_CMD_READ_PROM          0xA0

#define MS5611_OVERSAMPLING_4096 0x08
#define MS5611_OVERSAMPLING_2048 0x06
#define MS5611_OVERSAMPLING_1024 0x04
#define MS5611_OVERSAMPLING_512 0x02
#define MS5611_OVERSAMPLING_256 0x00

#define MS5611_VARIO_INTERVAL 500
#define MS5611_MEASUREMENT_INTERVAL 10

class MS5611 : public AbstractDevice, I2C, Vario
{
private:
    uint16_t C1_, C2_, C3_, C4_, C5_, C6_;
    uint32_t D1_, D2_;
    float temperature_ = 0, pressure_ = 0, P0_ = 0, altitude_ = 0, vario_ = 0;
    uint8_t device_, alphaTemp_, alphaDef_;
    void calcPressure();

public:
    MS5611(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef);
    void begin();
    void update();
    float *temperatureP();
    float *pressureP();
    float *altitudeP();
    float *varioP();
};

#endif