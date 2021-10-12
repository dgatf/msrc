#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>

#define OUTPUT_NORMAL 0
#define OUTPUT_HEX 1

#define MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000.0))
#define COMP_TO_MS(SCALER) ((SCALER * 1000.0) / F_CPU)
#define FIFO_SIZE 64

class Fifo
{
private:
    volatile uint8_t fifo[FIFO_SIZE];
    volatile uint8_t writePos = 0;
    volatile uint8_t readPos = 0;

public:
    Fifo();
    void write(uint8_t value);
    uint8_t read();
    uint8_t available();
    void reset();
};

class AbstractSerial
{
protected:
    uint8_t timeout_ = 0;

public:
    AbstractSerial();
    ~AbstractSerial();
    Fifo buffTx;
    Fifo buffRx;
    volatile uint16_t ts;
    virtual void begin(uint32_t baud, uint8_t format) = 0;
    virtual void initWrite() = 0;
    virtual void write(uint8_t data);
    virtual void writeBytes(uint8_t *buff, uint8_t size);
    virtual uint8_t read();
    virtual void readBytes(uint8_t *buff, uint8_t size);
    virtual uint8_t readFrame(uint8_t *buff);
    virtual uint8_t available();
    virtual uint8_t availableTimeout();
    virtual uint16_t getTimestamp();
    virtual void setTimeout(uint8_t timeout);

    void print(uint8_t value, uint8_t base = DEC);
    void print(int8_t value);
    void print(uint16_t value, uint8_t base = DEC);
    void print(int16_t value);
    void print(uint32_t value, uint8_t base = DEC);
    void print(int32_t value);
    void print(float value, uint8_t prec = 2);
    void print(char *value);
    void print(char data);
};

#endif