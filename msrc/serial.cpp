#include "serial.h"

Fifo::Fifo() {}

void Fifo::write(uint8_t value)
{
  if (((writePos - readPos) & (FIFO_SIZE - 1)) < (FIFO_SIZE - 1))
  {
    fifo[writePos] = value;
    writePos++;
    writePos &= (FIFO_SIZE - 1);
  }
}

uint8_t Fifo::read()
{
  if (readPos != writePos)
  {
    uint8_t value = fifo[readPos];
    readPos++;
    readPos &= (FIFO_SIZE - 1);
    return value;
  }
  return 0;
}

void Fifo::reset()
{
  readPos = writePos;
}

uint8_t Fifo::available()
{
  return (writePos - readPos) & (FIFO_SIZE - 1);
}

AbstractSerial::AbstractSerial() {}

AbstractSerial::~AbstractSerial() {}

void AbstractSerial::write(uint8_t data)
{
  buffTx.write(data);
  initWrite();
}

void AbstractSerial::writeBytes(uint8_t *buff, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++)
    write(buff[i]);
}

uint8_t AbstractSerial::read()
{
  return buffRx.read();
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
  uint8_t lenght = buffRx.available();
  readBytes(buff, lenght);
  return lenght;
}

void AbstractSerial::setTimeout(uint8_t timeout)
{
  timeout_ = timeout;
}

uint8_t AbstractSerial::available()
{
  return buffRx.available();
}

uint8_t AbstractSerial::availableTimeout()
{
  if ((uint16_t)micros() - ts > timeout_ * 1000)
    return buffRx.available();
  else
    return 0;
}

uint16_t AbstractSerial::getTimestamp()
{
  return ts;
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
    lenght = sprintf(buff, "%lu", value);
  else
    lenght = sprintf(buff, "%lX", value);
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

void AbstractSerial::print(char *buff)
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