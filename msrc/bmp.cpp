#include "bmp.h"

Bmp::Bmp(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : device_(device), alphaTemp_(alphaTemp), alphaDef_(alphaDef) {}

float Bmp::calcAltitude()
{
	if (P0_ == 500 && millis() > 2000)
		P0_ = pressure_;

	return 44330.0 * (1 - pow(pressure_ / P0_, 1 / 5.255));
}