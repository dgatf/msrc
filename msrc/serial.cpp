#include "serial.h"

AbstractSerial::AbstractSerial() {}

AbstractSerial::~AbstractSerial() {}

void AbstractSerial::reset()
{
    readPosRx = writePosRx;
}

void AbstractSerial::write(uint8_t data)
{
    if (((writePosTx - readPosTx) & (FIFO_SIZE - 1)) < (FIFO_SIZE - 1))
    {
        buffTx[writePosTx] = data;
        writePosTx++;
        writePosTx &= (FIFO_SIZE - 1);
    }
    initWrite();
}

void AbstractSerial::writeTx(uint8_t data)
{
    if (((writePosTx - readPosTx) & (FIFO_SIZE - 1)) < (FIFO_SIZE - 1))
    {
        buffTx[writePosTx] = data;
        writePosTx++;
        writePosTx &= (FIFO_SIZE - 1);
    }
}

void AbstractSerial::writeRx(uint8_t data)
{
    if (((writePosRx - readPosRx) & (FIFO_SIZE - 1)) < (FIFO_SIZE - 1))
    {
        buffRx[writePosRx] = data;
        writePosRx++;
        writePosRx &= (FIFO_SIZE - 1);
    }
}

void AbstractSerial::writeBytes(uint8_t *buff, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
        write(buff[i]);

    /*for (uint8_t i = 0; i < size; i++)
        writeTx(buff[i]);
    initWrite();*/ 
}

uint8_t AbstractSerial::read()
{
    if (readPosRx != writePosRx)
    {
        uint8_t value = buffRx[readPosRx];
        readPosRx++;
        readPosRx &= (FIFO_SIZE - 1);
        return value;
    }
    return 0;
}

uint8_t AbstractSerial::readTx()
{
    if (readPosTx != writePosTx)
    {
        uint8_t value = buffTx[readPosTx];
        readPosTx++;
        readPosTx &= (FIFO_SIZE - 1);
        return value;
    }
    return 0;
}

void AbstractSerial::readBytes(uint8_t *buff, uint8_t size)
{
    cli();
    for (uint8_t i = 0; i < size; i++)
        buff[i] = read();
    sei();
}

uint8_t AbstractSerial::readFrame(uint8_t *buff)
{
    uint8_t lenght = available();
    readBytes(buff, lenght);
    return lenght;
}

uint8_t AbstractSerial::available()
{
    return (writePosRx - readPosRx) & (FIFO_SIZE - 1);
}

uint8_t AbstractSerial::availableTx()
{
    return (writePosTx - readPosTx) & (FIFO_SIZE - 1);
}

void AbstractSerial::print(uint8_t value, uint8_t base)
{
    print((uint32_t)value, base);
}

void AbstractSerial::print(int8_t value)
{
    char buff[5];
    uint8_t lenght = sprintf(buff, "%i", value);
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(uint16_t value, uint8_t base)
{
    print((uint32_t)value, base);
}

void AbstractSerial::print(int16_t value)
{
    char buff[7];
    uint8_t lenght = sprintf(buff, "%i", value);
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(uint32_t value, uint8_t base)
{
    char buff[11];
    uint8_t lenght;
    if (base == HEX)
        lenght = sprintf(buff, "%lX", value);
    else
        lenght = sprintf(buff, "%lu", value);
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(int32_t value)
{
    char buff[12];
    uint8_t lenght = sprintf(buff, "%li", value);
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(float value, uint8_t prec)
{
    char buff[20];
    dtostrf(value, 1, prec, buff);
    uint8_t lenght = 0;
    while (buff[lenght] && lenght < 10)
        lenght++;
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(const char *buff)
{
    uint8_t lenght = 0;
    while (buff[lenght] && lenght < 32)
        lenght++;
    writeBytes((uint8_t *)buff, lenght);
}

void AbstractSerial::print(char data)
{
    write(data);
}