#include "ms4525do.h"

MS4525DOInterface::MS4525DOInterface() {}

bool MS4525DOInterface::begin()
{
  return true;
}

bool MS4525DOInterface::read()
{
  /*
uint8_t data[4];
data[0] = xx;
readBytes(_address, data, 4);

byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

  byte _status;
  unsigned int P_dat;
  float PR;
 
  while(1)
  {
    _status = fetch_pressure(&P_dat);
   
    switch(_status)
    {
      case 0: Serial.println("Read_MR.");
      break;
      case 1: Serial.println("Read_DF2.");
      break;
      case 2: Serial.println("Read_DF3.");
      break;
      default: Serial.println("Read_DF4.");
      break;
    }
   
   
    PR = (float)((P_dat - (0.1*16383)) / (0.8*16383)) ;
   
Serial.println(P_dat);
Serial.println(PR);
    Serial.print(" ");
   
    delay(1000);
  }
}

   
   
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
 
//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
 byte Temp_H = Wire.read();
 byte  Temp_L = Wire.read();
 
 
  _status = (Press_H >> 6) & 0x03;
      Press_H = Press_H & 0x3f;
      P_dat = (((unsigned int)Press_H) << 8) | Press_L;
      *p_P_dat = P_dat;
      return(_status);

 
 
  }*/
  return 0;
}
