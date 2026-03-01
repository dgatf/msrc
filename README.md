# MSRC – Multi Sensor Telemetry for RC (RP2040)

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/donate/?business=9GKNBVGHKXFGW&no_recurring=0&currency_code=USD)

**MSRC** is a lightweight, low-cost, and highly flexible telemetry system for RC models, based on the RP2040 microcontroller.

It allows you to collect data from multiple sensors and transmit it to your receiver using a wide range of telemetry protocols. MSRC is a DIY alternative to commercial solutions, offering significantly reduced weight and cost while maintaining high flexibility and expandability.

---

## Features

- Multi-sensor telemetry system based on RP2040
- Support for multiple receiver telemetry protocols
- Support for various ESC telemetry protocols
- Modular design (only use the sensors you need)
- Fully configurable via PC using MSRC Link
- Lightweight and cost-effective

---

## Supported Receiver Protocols

- FrSky SmartPort
- FrSky D
- FrSky FPort
- FrSky FBUS
- Spektrum XBUS
- Spektrum SRXL / SRXL2
- FlySky IBUS
- Futaba SBUS2
- Multiplex Sensor Bus (MSB)
- Jeti EX Bus
- Jeti EX Sensor
- Hitec
- CRSF
- Sanwa
- HoTT
- JR DMSS
- GHST

---

## Supported Sensors

### ESC Telemetry

- Hobbywing V3 / V4 / V5 / FlyFun
- Kontronik
- KISS (APD F, BLHeli32, Summit X)
- APD HV / UHV
- OMP M4
- ZTW
- Castle Link
- PWM / phase sensor ESCs
- Spektrum Smart ESC & Battery

### GPS

- Serial GPS (NMEA)

### Vario (I2C)

- BMP180
- BMP280
- MS5611

### Analog Sensors

- Voltage
- Temperature
- Current
- Airspeed (MPXV7002)

### Fuel

- Fuel flow meter (PWM pulses)
- Fuel tank pressure (XGZP68XXD)

### Other

- Up to 6 digital switches (GPIO)

---

## Getting Started

1. Download the latest firmware from the **Releases** section  
2. Flash the firmware to your RP2040 board  
3. Connect your sensors and receiver  
4. Configure the device using **MSRC Link**  
5. Check telemetry on your transmitter  

👉 For detailed instructions, see the [Wiki](https://github.com/dgatf/msrc/wiki)

---

## Documentation

Full documentation is available in the wiki:

👉 https://github.com/dgatf/msrc/wiki

---

## Support

- Report issues: https://github.com/dgatf/msrc/issues  
- Discussion forums:
  - https://www.rcgroups.com/forums/showthread.php?4088405-DIY-MSRC-Multi-sensor-telemetry-for-several-Rx-protocols#post48830585  
  - https://www.helifreak.com/showthread.php?t=908371  
  - https://www.openrcforums.com/forum/viewtopic.php?f=84&t=11911  

If you would like to add support for a new receiver protocol or sensor, feel free to open an issue.

---

## Donate

If you find this project useful, consider supporting its development:

👉 https://www.paypal.com/donate/?business=9GKNBVGHKXFGW&no_recurring=0&currency_code=USD
