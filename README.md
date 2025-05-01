# MSRC - Multi Sensor for RC - RP2040 - Smartport, Frsky D, XBUS, SRXL, IBUS, SBUS2, Multiplex Sensor Bus, Jeti Ex Bus, Hitec, CRSF, Sanwa, Hott, SRXL2, JR DMSS

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/donate/?business=9GKNBVGHKXFGW&no_recurring=0&currency_code=USD)

This is a DIY project to send sensors telemetry for a fraction of the weight and cost of the stock sensors. It adds support for several ESC telemetry protocols.

Compatible RX protocols:

- Frsky Smartport
- FrSky D
- Spektrum XBUS
- Spektrum SRXL
- Spektrum SRXL2
- Flysky IBUS
- Futaba SBUS2
- Multiplex Sensor Bus (MSB)
- Jeti Ex Bus
- Hitec
- CRSF
- Sanwa
- Hott
- JR DMSS

Compatible ESCs: Hobbywing V3/V4/V5/Flyfun, Kontronik Kosmic/Kolibri/JivePro, Castle Link, APD F/HV/UHV, phase sensor/PWM signal.

Compatible MCUs: RP2040. Any model is compatible. Recommended [RP2040 Zero](https://www.mischianti.org/wp-content/uploads/2022/09/Waveshare-rp2040-zero-Raspberry-Pi-Pico-alternative-pinout.jpg), for size and GPIO selection. Uunless you require GPIOs, then it is recommended to use RP2040 as this has pins 17 to 22 with a hole.   

Implemented sensors:

- ESC
  - ESCs with serial telemetry: Hobbywing V3/V4/V5/FlyFun<sup>(1)</sup>, Kontronik<sup>(2)</sup>, Kiss (APD F, blheli32, Summit X), APD HV/UHV
  - ESC with PWM signal or phase sensor
  - ESC Castle Link
  - Specktrum Smart ESC & Battery  
- GPS serial (NMEA)
- Vario (I2C sensors): BMP180, BMP280, MS5611
- Analog sensors: voltage, temperature, current, air speed (MPXV7002)  
- Fuel meter (PWM pulses)
- Fuel tank pressure (I2C sensor): XGZP68XXD
- 6 switches (gpio high or low)

All sensors are optional. Make the circuit with the desired sensors and enable them in the configuration. It can be configured from the PC with msrc_gui.

(1) If ESC doesn't arm, enable _ESC Init Delay_ or connect MSRC after the ESC is armed  
(2) Supported: Kosmic, Kolibri, JivePro. Not supported: Jive.

## Summary

[1.Binaries](#1-binaries)  
[2.Connections](#2-connections)  
[3. Power source](#3-power-source)  
[4. Flash firmware](#4-flash-firmware)  
[5. Configuration](#5-configuration)  
&emsp;[5.1 From PC - MSRC gui](#51-from-pc---msrc-gui)  
&emsp;[5.2. From transmitter using lua script (only Smartport)](#52-from-transmitter-using-lua-script-only-smartport)  
[6. Receiver protocol](#6-receiver-protocol)  
&emsp;[6.1 Smartport](#61-smartport)  
&emsp;[6.2 Frsky D](#62-frsky-d)  
&emsp;[6.3 Spektrum XBUS](#63-spektrum-xbus)  
&emsp;[6.4 Spektrum SRXL](#64-spektrum-srxl)  
&emsp;[6.5 Flysky IBUS](#65-flysky-ibus)  
&emsp;[6.6 Futaba SBUS 2](#66-futaba-sbus-2)  
&emsp;[6.7 Multiplex Sensor Bus (MSB)](#67-multiplex-sensor-bus-msb)  
&emsp;[6.8 Jeti Ex Bus](#68-jeti-ex-bus)  
&emsp;[6.9 Hitec](#69-hitec)  
&emsp;[6.10 Serial Monitor](#610-serial-monitor)  
&emsp;[6.11 CRSF](#611-crsf)  
&emsp;[6.12 Sanwa](#612-sanwa)  
&emsp;[6.13 Hott](#613-hott)  
&emsp;[6.14 Spektrum SRXL2](#614-spektrum-srxl2)  
&emsp;[6.15 JR DMSS](#615-jr-dmss)  
[7. Sensors](#7-sensors)  
&emsp; [7.1. ESC](#71-esc)  
&emsp;&emsp;[7.1.1. Available ESC telemetry](#711-available-esc-telemetry)  
&emsp;&emsp;[7.1.2. RPM multipliers](#712-rpm-multipliers)  
&emsp;&emsp;[7.1.3 Hobbywing Platinum V3](#713-hobbywing-platinum-v3)  
&emsp;&emsp;[7.1.4 Hobbywing Platinum V4 / FlyFun](#714-hobbywing-platinum-v4--flyfun)  
&emsp;&emsp;[7.1.5 Hobbywing Platinum V5](#715-hobbywing-platinum-v5)  
&emsp;&emsp;[7.1.6 Kontronik Kosmic/Kolibri/JivePro](#716-kontronik-kosmickolibrijivepro)  
&emsp;&emsp;[7.1.7 Kiss (APD F, blheli32, Summit X)](#717-kiss-apd-f-blheli32-summit-x)  
&emsp;&emsp;[7.1.8 APD UHV/HV series](#718-apd-uhvhv-series)  
&emsp;&emsp;[7.1.9. PWM signal](#719-pwm-signal)  
&emsp;&emsp;[7.1.10. Castle Link](#7110-castle-link)  
&emsp;&emsp;[7.1.11. Smart ESC and Battery](#7111-smart-esc-and-battery)   
&emsp;[7.2. Serial GPS](#72-serial-gps)  
&emsp;[7.3. Analog sensors](#73-analog-sensors)  
&emsp;&emsp;[7.3.1. Voltage divider](#731-voltage-divider)  
&emsp;&emsp;[7.3.2. Temperature (NTC)](#732-temperature)  
&emsp;&emsp;[7.3.4. Airspeed (MPXV7002)](#734-airspeed)  
&emsp;[7.4. Vario](#74-vario)  
&emsp;[7.5. Fuel Flow](#75-fuel-flow)  
&emsp;[7.6. Fuel pressure](#76-fuel-pressure)  
&emsp;[7.7. GPIOs](#77-gpios)  
[8. OpenTx sensors (Smartport)](#8-opentx-sensors-smartport)  
[9. Annex](#9-annex)  
&emsp;[9.1. ESC protocol specifications Hobbywing](#91-esc-protocol-specifications-hobbywing)  
&emsp;[9.2. ESC protocol specifications Castle Link](#92-esc-protocol-specifications-castle-link)  
&emsp;[9.3. ESC protocol specifications Kontronik](#93-esc-protocol-specifications-kontronik)  
&emsp;[9.4. Output PWM signal for FBL](#94-output-pwm-signal-for-fbl)  
&emsp;[9.5. ADC voltage](#95-adc-voltage)  
&emsp;[9.6. Analog voltage sensors. Voltage divider circuit](#96-analog-voltage-sensors-voltage-divider-circuit)  
&emsp;[9.7. Temperature sensors. Thermistors](#97-temperature-sensors-thermistors)  
&emsp;[9.8. Current](#98-current)  
&emsp;[9.9. Air Speed](#99-air-speed)  
&emsp;[9.10. Altitude](#910-altitude)  
[10. Change log](#10-change-log)  
[11. Support](#11-support)  

## 1. Binaries

File to flash to RP2040:
[MSRC-RP2040.uf2](https://drive.google.com/file/d/1D2POIdtwIJXGI-dHK78g2dTKvUnJVf65/view?usp=sharing)

PC application for configuration:
- Linux: [msrc_gui.AppImage](https://drive.google.com/file/d/16HKzOUz-pHu0FiNNqsd6UvzH1dXUbeao/view?usp=sharing)
- Windows: [msrc_gui.exe](https://drive.google.com/file/d/1f8ttDbwBAi8wI0ftW3WFy9LLaWnSEdvf/view?usp=sharing)
- macOS: [msrc_gui.dmg](https://drive.google.com/file/d/1hJD56Z4wOZTtu4Hz51h2b_V4zc7Qv3zn/view?usp=sharing). Built with macOS 14.7.2

## 2. Connections

Connections to RP2040 in the table bellow are GPIO numbers, which are the same for all RP2040 boards. Pin numbers are different depending on the board.

<center>

| Sensor/Receiver                           | Board GPIO|
| :---:                                     | :---:            |
| 3.3-5v                                    | 5v               |
| GND                                       | GND              |
| Smartport, SBUS, SRXL, IBUS, SB, Jeti Ex, CRSF, Sanwa, Hott, SRXL2  | 0<sup>(1)</sup> & 1 |
| Frsky D                                   | 0                |
| Hitec, XBUS SDA                           | 2<sup>(2)</sup>  |
| Hitec, XBUS SCL                           | 3<sup>(2)</sup>  |
| ESC serial, Serial monitor, Smart ESC     | 5                |
| Phase sensor (PWM in), Smart ESC          | 4                |
| Castle. Receiver signal                   | 4                |
| Castle. ESC signal                        | 5<sup>(2)</sup>  |
| GPS                                       | 6                |
| XBUS. NPN clock stretch<sup>(3)</sup>     | 7                |
| Sensor SDA                                | 8<sup>(2)</sup>  |
| Sensor SCL                                | 9<sup>(2)</sup>  |
| PWM out                                   | 10               |
| Fuel meter (PWM in)                       | 11               |
| Throttle PWM (Smart ESC)                  | 12               |
| Reverse PWM (Smart ESC)                   | 13               |
| Voltage                                   | 26               |
| Current                                   | 27               |
| NTC                                       | 28               |
| Airspeed                                  | 29               |
| GPIOs                                     | 17 to 22         |

</center>

(1) with 100Ω resistor. This resistor is optional as RP2040 has internal protection resistors for GPIOs  
(2) Pullups. See [6.3 Spektrum XBUS](#63-spektrum-xbus) and [6.9 Hitec](#69-hitec)     
(3) Optional  

Status led of the board blinks when sending telemetry. If it doesn't blink check connections and config.


<p align="center"><img src="./images/rp2040_zero_pinout.jpg" width="500"><br>
  <i>RP2040 Zero pinout</i><br><br></p>

## 3. Power source

RP2040 Zero has to be powered with 5v or less. For higher voltage, a voltage regulator is needed.

It can be powered from the telemetry port or from BEC. Some receivers deliver 5v or less at the telemetry port, even if BEC voltage is higher than 5v. Check that voltage supplied to RP2040 is not higer than 5v.  

## 4. Flash firmware

Press BOOT button during startup and paste the binary file [MSRC-RP2040.uf2](https://drive.google.com/file/d/1U6SIOXrxWNSM_oTFpoiJkpZdhAzVZ5Iy/view?usp=sharing) to RP2040 folder

If you want to build the firmware for the RP2040 yourself:

Install [RP2040 SDK](https://github.com/raspberrypi/pico-sdk)  

<code>git clone --recurse-submodules https://github.com/dgatf/msrc.git  
cd board  
mkdir build  
cd build  
cmake ..  
make</code>

File to flash: _project/MSRC-RP2040.uf2_

## 5. Configuration

### 5.1. From PC - MSRC gui

Connect RP2040 to USB and update config with msrc_gui. [msrc_gui.AppImage](https://drive.google.com/file/d/1mvA-3e6PpEAfOVxoN4NynNnP9kEp_QXQ/view?usp=sharing) for Linux, [msrc_gui.exe](https://drive.google.com/file/d/1q56j305oKkxwy4nNbp1RR60IOu2S3hbO/view?usp=sharing) for windows or [msrc_gui.dmg](https://drive.google.com/file/d/1zEmVQUYT4Z_JjZQU5CpxyqsHoHrJYY26/view?usp=sharing) for macOS. After update reboot RP2040 manually.

<p align="center"><img src="./images/msrc_gui.png" width="600"><br>

Debug mode can be enabled and viewed with msrc_gui.

<p align="center"><img src="./images/msrc_gui_debug.png" width="600"><br>

If you want to build msrc_gui:

- Install [QT](https://www.qt.io/)
- Install Qt serial port library (Use _Maintenance Tool_ or in Ubuntu *sudo apt install libqt5serialport5-dev*)
- _cd msrc_gui_
- _mkdir build_
- _cd build_
- _qmake .._
- _make_

### 5.2. From transmitter using lua script (only Smartport)

It is recommended to update config with *msrc_gui* as not all parameters are implemented in lua script.

The configuration is modified with a lua script (X7, X9, X-lite and Horus with openTx 2.2 or higher)

<p align="center"><img src="./images/lua_x7.png" height="128">   <img src="./images/lua_x9.png" height="128">   <img src="./images/lua_x10.png" height="200"></p>

Copy the file msrc.lua to the SCRIPTS/TOOLS folder. (if using older openTx 2.2 copy to SCRIPTS folder and execute by long press)

Options:

- ESC protocol. HobbyWing Platinum V3, HobbyWing Platinum V4/Hobbywing Flyfun V5 or PWM signal
- Voltage. Enable/disable analog voltage divider 1
- Ntc. Enable/disable analog thermistor 1
- Current. Enable/disable analog current
- Airspeed. Enable/disable analog airspeed sensor
- PWM out. Enable/disable analog a PWM signal from RPM values from ESC serial
- GPS.  Enable/disable serial GPS
- Averaging queue size: 1 to 16
- Refresh rate (ms): 0 to 1600
- I2C. Sensor type and address

## 6. Receiver protocol

The following Rx protocols are supported:

- Frsky Smartport: inverted serial, 57600 bps
- Frsky D: inverted serial, 9200 bps
- Spektrum XBUS: I2C
- Spektrum SRXL V5: serial, 115200 bps
- Flysky IBUS: serial, 115200 bps
- Futaba SBUS2: inverted serial, 100000 bps
- Multiplex Sensor Bus: serial, 38400 bps
- Jeti Ex Bus: serial 125000, 250000 bps
- Hitec: I2C
- Serial Monitor. This is not a Rx protocol, but a serial monitor on GPIO5 
- CRSF: serial, 416666 bps  
- Sanwa: serial, 115200 bps
- Hott: serial 19200 bps  
- Spektrum SRXL2: serial 115200 bps  
- JR DMSS: serial 250000, 2 stop bits

### 6.1 Smartport

Connect MSRC to Smartport.  

### 6.2 Frsky D

Connect MSRC the telemetry port.  

### 6.3 Spektrum XBUS

Auto-config may be used to detect the new sensors.


<p align="center"><img src="./images/xbus_connector.png" width="300"><br>
  <i>XBUS port</i><br><br></p>

#### Pullups

For older telemetry receivers with only XBUS port like TM1000 and TM1100, pull up resistors are needed. For newer receivers with both XBUS and SRXL2 ports, pull up resistors are not needed. In either case it has to be the correct option, otherwise XBUS won't work. So, if no telemetry shown in transmitter try the other option (with or without pull ups). One of the two posibilities will work. (Thanks RCG_KevinB from RCGroups for solving this issue). 

#### Boot time and clock stretch

With old receivers this does not affect, but newer receivers boot faster and could be an issue.  

If no telemetry is shown, may be MSRC is booting too slow and the first poll from the receiver is not answered. There are several ways to fix this:

1. Power on the receiver after MSRC has started
3. I2C clock stretch. Pull down the SCL line until MSRC has started, then open the switch. You can use a manual switch or a NPN transistor (e.g. PN2222ABU). If using a transistor you need to enable _XBUS Clock Stretch_ to open the transistor switch after boot. If using manual switch, open the switch after boot, to finish the clock stretch

<p align="center"><img src="./images/xbus_switch.png" width="300"><br>
  <i>Clock stretch XBUS with manual switch</i><br><br></p>

### 6.4 Spektrum SRXL

Spektrum SRXL is bidirectional with telemetry, other SRXL are unidirectional. See [SRXL](https://wiki.beastx.com/index.php?title=SRXL_-_Serial_Receiver_Link_protocol/en).  

<p align="center"><img src="./images/spektrum_srxl.jpg" width="300"><br>
  <i>Spektrum SRXL</i><br><br></p>

### 6.5 Flysky IBUS

Connect MSRC to sensor port.  

### 6.6 Futaba SBUS 2

Slots sensor mapping for Futaba transmitters:

<center>

| Slot   | Sensor                               |
| :----: | :----------------------------------: |
|0       | RX voltage (reserved)                |
|1       | Temperature 1 (SBS-01T/TE)           |
|2       | RPM (type magnet)(SBS-01RB/RM/RO)    |
|3-4     | Vario-F1672                          |
|6-7     | Voltage (SBS-01V)                    |
|8-15    | GPS-F1675 <sup>(2)</sup>                         |
|16      | Air speed (SBS-01TAS)                |
|17-23   | Unused                               |
|24-26   | Current 1 (SBS-01C)                  |
|27-29<sup>(2)</sup>| Current 2 (SBS-01C)                  |
|30<sup>(2)</sup>   | Temperature 2 (SBS-01T/TE)           |
|31      | Unused                               |

</center>

(1) Do not select default GPS  
(2) Non default slots

Select protocol: FASSTest 18CH or T-FHSS

Connect to SBUS2 port

### 6.7 Multiplex Sensor Bus (MSB)

Connect to *Sensor* port

<p align="center"><img src="./images/multiplex_sensorbus.jpg" width="300"><br>
  <i>Sensor bus port</i><br><br></p>

### 6.8 Jeti Ex Bus

Configure receiver pin (E1 or E2) as Ex Bus. The maximum number of sensors (values) is 15. If more sensors are needed, use an another MSRC board and connect to E2 or to a *Jeti Telemetry Expander*.  

### 6.9 Hitec

Use 6k pullup resistors.  

If Vin is a regulated 5V source (e.g. receiver, BEC), pull ups can be connected to Vin/RAW instead Vcc.  

Wire colors in the images bellow:

- Yellow wire is SCL (gpio3)
- Red is SDA (gpio2)
- Black is GND

<p align="center"><img src="./images/hitec_7.jpg" width="300"><br>
  <i>Hitec Optima 7</i><br><br></p>

<p align="center"><img src="./images/hitec_9.jpg" width="300"><br>
  <i>Hitec Optima 9</i><br><br></p>

### 6.10 Serial Monitor

This is not a Rx protocol. No sensors are enabled.  

Enable a serial monitor on GPIO5 with the selected parameters.  

### 6.11 CRSF

Telemetry for CRSF does not cover all sensors in MSRC. Available sensors with CRSF are: GPS, vario and battery (voltage, current and consumption).  

### 6.12 Sanwa

Available sensors:

- ESC temperature.
- Motor temperature. In case the ESC provides BEC temperature, it is linked to this telemetry value. To measure motor temperature, enable analog temperature and use a ntc termistor attached to the motor. It overrides BEC temperature if available from the ESC.   
- Rpms.
- Voltage.

### 6.13 Hott

Coonect to the telemetry port marked with a T. Depending on the model, there is a dedicated port for telemetry (e.g. GR-32) or it uses a servo channel (e.g. GR-12).  

### 6.14 Spektrum SRXL2

If the receiver goes into bind mode or there is no telemetry, add a 10k pullup to the signal line.

See [SRXL2 specifications](https://github.com/SpektrumRC/SRXL2/blob/master/Docs/SRXL2%20Specification.pdf).

<p align="center"><img src="./images/srxl2_connectors.png" width="500"><br>
  <i>SRXL2 connectors</i><br><br></p>

### 6.15 JR DMSS

Connect to XBUS port.  

## 7. Sensors

Available sensors:

- ESC
  - ESCs with serial telemetry: Hobbywing V3/V4/V5/Flyfun, Kontronik<sup>(2)</sup>, Kiss (APD F, blheli32, Summit X), APD HV/UHV  
  - ESC with PWM signal or phase sensor
  - ESC Castle Link
- GPS serial (NMEA)
- I2C sensors (vario): BMP180, BMP280, MS5611
- Analog sensors: voltage, temperature, current, air speed

All sensors are optional. Make the circuit with the desired sensors and enable them in the configuration. It can be configured from the PC with msrc_gui.

### 7.1. ESC

#### 7.1.1. Available ESC telemetry

| Model              | RPM         | Throttle    | Motor PWM   | Voltage     | Current   | Voltage BEC | Current BEC | Temperature 1 | Temperature 2 | Ripple Volt |
| ------------------ | :---------: | :---------: | :---------: | :---------: | :-------: | :---------: | :---------: | :-----------: | :-----------: | :---------: |
| Hobbywing V3       | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:<sup>(1)</sup>      |             |           |             |             |               |               |             |
| Hobbywing V4/V5<sup>(4)</sup><sup>(5)</sup> | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:<sup>(2)</sup>    |             |             | :heavy_check_mark: FET       | :heavy_check_mark: BEC       |             |
| Castle Link        | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:       | :heavy_check_mark:<sup>(3)</sup>      | :heavy_check_mark:<sup>(3)</sup>      | :heavy_check_mark:           |               | :heavy_check_mark:         |
| Kontronik        | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:       | :heavy_check_mark:      | :heavy_check_mark: | :heavy_check_mark: Power amp | :heavy_check_mark: BEC     |         |
| APD F | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup> | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:    |             |             | :heavy_check_mark:       |        |             |
| APD HV/UHV | :heavy_check_mark:         | :heavy_check_mark:<sup>(1)</sup>  | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:    |             |             | :heavy_check_mark:      |       |             |
| Smart ESC&Bat(6)      | :heavy_check_mark: | :heavy_check_mark:<sup>(1)</sup> | :heavy_check_mark:<sup>(1)</sup>      | :heavy_check_mark:         | :heavy_check_mark:    | :heavy_check_mark:<sup>(3)</sup> | :heavy_check_mark:<sup>(3)</sup> | :heavy_check_mark:      |       |             |


(1) Available but not forwarded to smartport  
(2) For 80A models and higher  
(3) Not available in all models  
(4) Sensors varies depending on model and firmware. See [annex](#Hobbywing-V4-V5)  
(5) Upgrade to latest firmware version. But note VBAR versions of firmware are not compatible. Install a non VBAR version of the ESC firmware  
(6) See [Smart ESC and Battery](#7111-smart-esc-and-battery) for battery telemetry  

If voltage is available the cell voltage average is calculated for 3S,4S,5S,6S,7S,8S,10S and 12S batteries. 10 seconds after power on the number of cells is autodetected. Average cell voltage to be >3.8v for correct cell count

If current is available, battery consumption is calculated in mAh

#### 7.1.2. RPM multipliers

Adjust RPMs in *msrc_gui*:

- Set the number of pair of poles of the motor, _Pair of poles_
- For helis also set the pinion and main gear teeth: _Pinion teeth_, _Main teeth_

Alternatively this can be done in the transmitter

#### 7.1.3. Hobbywing Platinum V3

Serial protocol: 19200 baud.  

Telemetry port is the program port. It is a servo male plug on the ESC

#### 7.1.4. Hobbywing Platinum V4 / FlyFun

Serial protocol: 19200 baud.  

HW FlyFun. If ESC doesn't arm, enable _ESC Init Delay_ or connect MSRC after the ESC is armed.  

Optionally, for Hobbywing Flyfun and APD F-series, a PWM signal can be generated from the RPM telemetry value.  

Check your ESC firmware is not VBAR version. Telemetry port is the program port. It is a servo male plug on the ESC

- Values for current when throttle is low (up to 25%, depending on model) may not be reliable. If getting high or noisy current values when throttle is low, adjust _Current_thresold_. Below this throttle percentage (0-100%), current values will be 0A
- Set _Max Current_ to the peak current of the ESC (eg: 80A: _Max Current_ 100)
- Adjust _Voltage Divisor__ and _Current Multiplier__, depending on model:
 
<center>

| Cells range | Voltage divisor |
| :---: | :---: |
| 3-6S (LV) | 11 |
| 3-8S (LV v2) | 15.75 |
| 5-12s (HV) | 21 |
| 6-14s (HV v2) | unknown |

| Amperage | Current multiplier |
| :---: | :---: |
| FlyFun 60A | 670 |
| FlyFun 80A | 320 |
| 100A | 440<sup>(1)</sup> |
| Platinum V4 120A | 400 |
| 130A | 350<sup>(1)</sup> |
| 150A | 310<sup>(1)</sup> |
| 160A | 290<sup>(1)</sup> |
| Platinum V4 200A | 240 |

</center>

(1) Extrapolated from confirmed models. If you find discrepancy adjust gain parameter and send gain and ESC model to update the table

#### 7.1.5. Hobbywing Platinum V5

Serial protocol: 115200 baud.  

#### 7.1.6. Kontronik Kosmic/Kolibri/JivePro

Serial protocol: 115200 baud, even parity. 

Supported models: Kosmic, Kolibri, JivePro. Not supported: Jive.  

#### 7.1.7 Kiss (APD F, blheli32, Summit X)

Any ESC with Kiss telemetry: APD F, blheli32, Summit X.  

__APD F__

Telemetry port for APD F: pin T

Types of telemetry:

- *RPM output* it is a PWM signal with the RPMs. Select PROTOCOL_PWM and connect to pin PWM in as noted in [connections table](#1-connections)

- *PWM telemetry*. For this type of telemetry you need to flash [beta firmware](https://docs.powerdrives.net/products/firmware-installation/f_series-beta-configurable-firmware). It provides serial telemetry with RPMs, voltage, current and temperature. Select PROTOCOL_APD_F. Connect ESC pin T to pin Rx as defined in [connections table](#1-connections).  

#### 7.1.8 APD UHV/HV series

Serial telemetry with RPMs, voltage, current and temperature.

#### 7.1.9. PWM signal

If the ESC have a PWM signal or a phase sensor is installed, RPMs can be measured. If ESC have both serial and PWM signal, like Hobbywing V4/V5, then PWM signal is not needed for telemetry

#### 7.1.10. Castle Link

The telemetry is send over the ESC signal. *Castle Link* has to be enabled in the ESC config.  

#### 7.1.11. Smart ESC and Battery

Spektrum Smart ESC & Battery uses SRXL2 protocol.  

Connections:

- Smart ESC signal -> GPIOs 4 & 5
- Throttle PWM -> GPIO 12
- Reverse PWM -> GPIO 13 (optional)

Smart Battery sends telemetry for: current, consumption, cells voltage and temperature.  

### 7.2. Serial GPS

Serial GPS (NMEA protocol) is supported. Select the same baudrate as the GPS module.  

### 7.3. Analog sensors

#### 7.3.1. Voltage divider

Calibrate voltage analog sensors with _Voltage multiplier_. Or from opentx, but it is recommended in order to increase sensor resolution

Multiplier = (R1+R2)/ R2

#### 7.3.2 Temperature

Use NTC thermistors. No need to calibrate. For fine tuning adjust in ntc.h: NTC_R_REF, NTC_R1, NTC_BETA

#### 7.3.3. Current

Calibrate current analog sensor from msrc_gui. Or from opentx, but it is recommended in order to increase sensor resolution

Set sensor type:

- Hall effect (ACS758, Amploc). Multiplier = 1000 / sensitivity(mV/A). Select auto-offset or set zero current output voltage (Viout), which us the voltage at current 0

- Shunt resistor sensor. Multiplier = 1000 / (ampgain * resistor(mΩ))

#### 7.3.4. Airspeed

Use differential pressure analog sensor MPXV7002. Vcc for the airspeed sensor must be 5V. The maximum readable output the sensor is 3.3V due to the rp2040 ADC range : 0-3-3V
The limits the maximum speed to about 120km/h. Over 3.3V output the reading is the same as for 3.3V. It won't damage the rp2040 as Vout current is 0.1mA.  
If higher speed measurement (up to 170km/h) is needed, an stepdown has to be added. But this is still not added to the code.  

Relation between Vout and delta pressure.

<p align="center"><img src="./images/MPXV7002.png" width="500"><br>
  <i>MPXV7002</i><br><br></p>

If a barometer sensor is installed, you'll get more accurate airspeed values as air density is calculated with actual temperature and pressure. Otherwise following default values are used: 20ºC and 101325Pa.  

Parameters:

slope = Vcc/5. Nominal value is 1  
offset = ±6.25% VFSS = ±0.25. Nominal value is 0  

Transformation formula:

<img src="https://latex.codecogs.com/svg.latex?V_{out}=V_{cc}(2\Delta{P}+0.5)\pm{6.25\%VFSS}" title="Vout = Vcc x (0.2 x ΔP + 0.5) ±6.25% VFSS" />

Transformation formula with MSRC parameters:

<img src="https://latex.codecogs.com/svg.latex?\Delta{P}=\frac{V_{out}-offset}{slope}-2.5" title="ΔP = (Vout - offset) / slope - 2.5" />

Air density calculation:

<img src="https://latex.codecogs.com/svg.latex?\rho=\frac{P}{R\cdot{T}}" title="ρ = P / (R * T(Kelvin))" />

R -> dry air constant  

Airspeed calculation:

<img src="https://latex.codecogs.com/svg.latex?TAS=\sqrt{\frac{2\Delta{P}}{\rho}}" title="TAS(m/s) = sqrt(2 * ΔP / air_density)" />

TAS -> airspeed (m/s)  

### 7.4. Vario

The following I2C barometer sensors are suported:

- Barometer: BMP180, BMP280, MS5611

Vspeed is calculated from air pressure and temperature.   

### 7.5. Fuel flow

This sensor is available for: XBUS, Smartport and JetiEx.  

Fuel flow sensor with PWM pulses are supported. It is neded to set the parameter ml/pulse.

If this parameter is unknown, to calibrate:

- Measure the weight of the tank or the model
- Enable Fuel Flow sensor in msrc_gui
- Then either:
  - Set the parameter ml/pulse to 1 and then the number of pulses will be the consumption in your transmitter 
  - Enable log in msrc_gui and note the number of pulses
- After a while, stop the engine. Note the last number for pulses
- Weight the tank or model again and use the folloing formula to obtain the parameter:

<img src="https://latex.codecogs.com/svg.latex?ml/pulse=\frac{\Delta{Weight}\cdot{\rho}}{pulses}" title="ml/pulse=ΔWeight x ρ /pulses" />

ρ -> fuel density (g/m<sup>3</sup>)  
ΔWeight -> g

### 7.6. Fuel pressure
 
Use XGZP68XXD. Set K parameter with sensor maximum pressure, depending on the model.

Available for: XBUS, SRXL, SRXL2 and Jeti Ex.   

### 7.7. GPIOs
 
Enable GPIOs 17 to 22 to read the pin state (high or low). It is recommended to use the RP2040-zero as this has those pin with a hole.   

Available for Smartport with dataId 0x51nn, where nn is the GPIO number.  

If using Ethos firmware, you have to create a DIY sensor. Set *Physical ID* as *Sensor Id*. And set *Application Id* to 51nn. Repeat for ech enabled GPIO.    

<p align="center"><img src="./images/ethos_gpio.png" width="500"><br>
  <i>Ethos DIY Sensor</i><br><br></p>


## 8. OpenTx sensors (Smartport)

The default sensor id is 10. This can be changed with [change_id_frsky](https://github.com/dgatf/change_id_frsky)

Depending on your configuration some the following sensors will be available in Opentx. After configuration go to sensors screen and update with *Search new sensors*
 
ESC:

- ESC RPM: Erpm (0x0b60)
- ESC voltage: EscV (0x0b50)
- ESC cell average: Cells (0x300)
- ESC current: EscA (0x0b50)
- ESC consumption: EscC (0x0b60)
- ESC temp FET (HW) or ESC temp linear (Castle): EscT (0x0b70)
- ESC temp BEC (HW) or ESC temp NTC (Castle): EscT (0x0b71)
- ESC ripple voltage: EscV (0x0b51)
- ESC BEC voltage: BecV (0x0e50)
- ESC BEC current: BecC (0x0e50)

GPS:

- GPS Lat/Lon: GPS (0x800)
- GPS altitude: GAlt (0x820)
- GPS speed: GSpd (0x820)
- GPS compass: Hdg (0x840)
- GPS date/time: Date (0x850)
- GPS sats: 0x5103  
Calculated:
- Vario: 0x111
- Distance to home: 0x5104

Analog:

- Voltage: A3 (0x0900)
- Thermistor: Tmp1 (0x0400)
- Current: Curr (0x020f)
- AirSpeed: ASpd (0x0a00)

I2C:

- Altitude: Alt (0x0820)
- Temperature: T1 (0x0401, 0x0402)  
Calculated:
- Vario: 0x110

Some of the sensors have to be adusted in openTx

### 8.1. Adjust RPM sensor (Erpm)

If not adjusted in MSRC config you can adjust in the sensor parameters in opentx:

- Blades/poles: number of pair of poles * main gear teeth  
- Multiplier: pinion gear teeth

### 8.2. Adjust voltage sensors (A3, A4)

Remark: Instead of adjusting the sensor in opentx, it is recommended to use _Voltage multiplier_ to increase the sensor resolution.

Measure the voltage of the battery with a voltmeter and adjust *Ratio* in A3, A4 sensor.

### 8.3. Adjust analog current sensor (Curr)

Remark: Instead of adjusting the sensor in opentx, it is recommended to use _Current multiplier_ to increase the sensor resolution

If using a hall effect sensor, adjust the ratio: *25.5 x 1000 / output sensitivity (mV/A)*

To calculate the battery consumption add a new sensor:

- Type: Calculated
- Formula: Consumption
- Sensor: Curr

### 8.4. Calculate current consumption

Battery consumption is calculatd since MSRC v0.9

Alternatively can be calculated by adding a calculated sensor in openTx:

- Type: _Calculated_
- Formula: _Consumption_
- Sensor: _EscA_ or _Curr_


## 9. Annex

### 9.1. ESC protocol specifications Hobbywing

Serial parameters:

- 19200 bps
- 1 packet every 20 ms
- Big endian


#### 9.1.1. Hobbywing V3

| Byte  | 1                   | 2                | 3                | 4                | 5             | 6              | 7            | 8            | 9           | 10          |
| ----- | :-----------------: | :--------------: | :--------------: | :--------------: | :-----------: | :------------: | :----------: | :----------: | :---------: | :---------: |
| Value | Package Head (0x9B) | Package Number 1 | Package Number 2 | Package Number 3 | Rx Throttle 1 | Rx Throttle  2 | Output PWM 1 | Output PWM 2 | RPM Cycle 1 | RPM Cycle 2 |

*RPM = 60000000 / RPM Cycle*

thr, pwm: 0-255 (8bits)

#### 9.1.2. Hobbywing V4 V5

| Byte  | 1     | 2     | 3     | 4     | 5     | 6     | 7     | 8     | 9     | 10    | 11    | 12    | 13    | 14    | 15    | 16    | 17    | 18    | 19    |
| ---   | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Value | Package Head (0x9B) | Package Number 1 |	Package Number 2 | Package Number 3	| Rx Throttle 1	| Rx Throttle  2 | Output PWM 1 | Output PWM 2	| RPM 1 | RPM 2	| RPM 3	| Voltage 1 |	Voltage 2	| Current 1	| Current 2	| TempFET 1	| TempFET 2	| Temp 1 |	Temp 2

thr, pwm: 0-1024 (10bits)

Voltage, current and temperature are raw sensor data. Actual values requires transformation. Depending on the model, sensors are different

Before throttle is raised from 0, signature packets are sent between telemetry packets. This is used to identify the hardware and firmware of the ESC

Examples:

| Model\Byte    | 1     | 2     | 3     | 4     | 5     | 6     | 7     | 8     | 9     | 10    | 11    | 12    | 13    |
| ----------    | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| V4LV25/60/80A | 0x9B  | 0x9B  | 0x03  | 0xE8  | 0x01  | 0x08  | 0x5B  | 0x00  | 0x01  | 0x00  | 0x21  | 0x21  | 0xB9  |
| V4HV200A OPTO | 0x9B  | 0x9B  | 0x03  | 0xE8  | 0x01  | 0x02  | 0x0D  | 0x0A  | 0x3D  | 0x05  | 0x1E  | 0x21  | 0xB9  |
| V5HV130A OPTO | 0x9B  | 0x9B  | 0x03  | 0xE8  | 0x01  | 0x0B  | 0x41  | 0x21  | 0x44  | 0xB9  | 0x21  | 0x21  | 0xB9  |


### 9.2. ESC protocol specifications Castle Link

For best accuracy RX pulse input is captured with a timer interrupt and ESC pulse output is produced by hardware PWM. Maximum latency is 20ms

See [Castle Link Live](https://dzf8vqv24eqhg.cloudfront.net/userfiles/4671/6540/ckfinder/files/Product%20Manuals/Accessories%20and%20replacement%20parts/castle_link_live_2_0.pdf?dc=201606221536-537)

### 9.3. ESC protocol specifications Kontronik

Supported models: Kosmic, Kolibri, JivePro. Protocol [specs](https://www.kontronik.com/fileadmin/kontronik-sobek/Public/Content/Images/Content/Downloads/Software/Kontronik_TelMe_V4.12_1.12_EN.pdf)

Note that there is another protocol used by Kontronik Jive at 9600 that it is not supported. Info [here](https://www.helifreak.com/showthread.php?t=721180)

### 9.4. Output PWM signal for FBL

For best accuracy PWM signal output for FBL is produced by hardware PWM from serial RPM values. Maximum latency is 40ms

### 9.5. ADC voltage

To obtain the voltage at the analog pin it is required the ADC bits (b) and the Vref:

<img src="https://latex.codecogs.com/svg.latex?V_o=V_{ref}*\frac{Raw}{2^b}" title="Vo = Vref * Raw / 2^bits" /><br>

### 9.6. Analog voltage sensors. Voltage divider circuit

Two battery voltages can be measured through the analog pins A2 and A3
Metal resistors are recommended as gives more accurate readings (0.1W or higher)
Maximum voltage is 3.3V

<p align="center"><img src="./images/Resistive_divider.png" width="200"></p>

To select R values apply formulas:

<img src="https://latex.codecogs.com/svg.latex?V_o=V_{in}\frac{R_2}{R_1+R_2}<3.3Vor5V" title="Vo = Vi\*R2/(R1+R2) < 3.3V or 5V" /><br>

<img src="https://latex.codecogs.com/svg.latex?Z=\frac{1}{\frac{1}{R_1}+\frac{1}{R_2}}<10K" title="Z = 1/((1/R1)+(1/R2)) < 10K" />


For 6S battery (or lower) and Pro Mini 3.3v:

 - R1 68k
 - R2 10k

In this case ratio is 7.8

If more than 6S change R values or you may burn the board!

### 9.7. Temperature sensors. Thermistors

Two temperature sensors can be installed through the analog pins A0 and A1
Temperature is measured with NTC thermistors (100k). Adjust thermistor Beta in ntc.h if needed (NTC_BETA, default is 4190). Sensor output in Celsius

<p align="center"><img src="./images/ntc.gif" width="200"></p>

To obtain the thermistor resistance:

<img src="https://latex.codecogs.com/svg.latex?R_t=\frac{V_o*R_s}{(V_{in}-V_o)}" title="Rt = Vo * Rs / (Vin - voltage))}" />

And temperature with Beta formula:

<img src="https://latex.codecogs.com/svg.latex?T=\frac{1}{\frac{ln\frac{R_t}{R_{ref}}}{\beta}+\frac{1}{T_{ref}}}" title="T = 1/[ln(Rt/Rref)/B+1/Tref]" />

Or with Steinhart and Hart Equation if data is available:

<img src="https://latex.codecogs.com/svg.latex?T=\frac{1}{A+B*ln\frac{R_t}{R_{ref}}+C*ln(\frac{R_t}{R_{ref}})^2+D*ln(\frac{R_t}{R_{ref}})^3}" title="T = 1/[A+Bln(Rt/Rref)+Cln(Rt/Rref)²+Dln(Rt/Rref)³]" />

### 9.8. Current

#### 10.8.1. Hall effect

Hall effect sensors. Induced magnetic field is transformed into voltage. They are caracterized by their sensitivity

<img src="https://latex.codecogs.com/svg.latex?I=\frac{V_o}{S}" title="I=Vo/S" />

#### 10.8.1. Shunt resistor

The voltage drop in the shunt resistor is amplified by a differential amplifier to obtain Vo

<img src="https://latex.codecogs.com/svg.latex?I=\frac{V_o}{A_d*R_s}" title="I=Vo/Ad*Rs" />

<p align="center"><img src="./images/High-Side-Current-Sensing.png" width="200"></p>

### 9.9. Air Speed

### 9.10. Altitude

Altitude is calculated using the barometric formula:

<img src="https://latex.codecogs.com/svg.latex?h=(1-\frac{P}{Po}^{\frac{RL}{gM}})\frac{T}{L}" title="h=(1-P/Po^RL/gM)T/L" />

*R = universal gas constant: 8.3144598 J/(mol·K)  
g = gravitational acceleration: 9.80665 m/s2  
M = molar mass of Earth's air: 0.0289644 kg/mol  
L = temperature lapse rate (K/m): 6.5 C/km  
T = temperature at h (K)  
Po = pressure at ground (Pa)*  

5 seconds after boot, pressure reference, Po, is set


## 10. Change log

v1.3

- Added GPIOs 17 to 22 for input
- Added support for Spektrum Smart ESC & Battery   

[v1.2.2](https://github.com/dgatf/msrc/releases/tag/v1.2.2)

- Fixed Hitec protocol
- Improved vspeed accuracy
- SRXL2. Ask for re-handshake on reboot

[v1.2.1](https://github.com/dgatf/msrc/releases/tag/v1.2.1)

- Fixed serial half duplex from interrupts
- Fixed SRXL2 for no remote receivers

[v1.2](https://github.com/dgatf/msrc/releases/tag/v1.2)

- Added github actions for continuous deployment
- Added support for macOS
- Added support for fuel tank pressure sensor (XGZP68XXD)
- Added support for fuel flow sensor with pulses output  
- Added support for Hott telemetry  
- Added support for Spektrum SRXL2 telemetry  
- Implemented serial half duplex
- Added support for JR DMSS telemetry. Thanks to *Gorochan* and *gardfiel2412* from RCGroups  
- Added two decimals for current sensitivity and offset  

[v1.1](https://github.com/dgatf/msrc/releases/tag/v1.1)

- Fixed Xbus negative values and improve vspeed accuracy  
- Updated sdk and third party libraries to support new hardware  
- Fixed analog current  
- Added support for CRSF protocol  
- Added support for Sanwa protocol   
- Improved airspeed calculation and linked to barometer if installed to get more accurate aurspeed value   
- msrc_gui. Manage different config versions  

[v1.0](https://github.com/dgatf/msrc/releases/tag/v1.0)

- Ported to RP2040

[v0.9](https://github.com/dgatf/msrc/tree/v0.9)

- HW V4/V5. Added throttle threshold and maximum current filters for current sensor. Values for current when throttle is low may not be reliable, depending on model
- Added consumption calculation. Thanks MJ666
- Fixed Jeti Ex bug
- HW V4/V5. Only one HW V4/V5 protocol. Voltage and current sensor parameters to be set manually
- Added RPM multipliers. Motor poles and gears (for helis) 
- Changed serial drivers timeout to microseconds for fast protocols (e.g. Jeti Ex)
- Allow additional Rx protocols & boards to be used with serial ESC and GPS. Use software serial only for Rx protocols, not sensors
- GPS. Added HDOP, sats, vario and distance to home
- IBUS. Fixed S32 type bug (GPS values)
- XBUS. Added analog voltage2/ntc2 sensor
- BMP280 and GPS. Added vertical speed calculation (vario)
- Current analog. Added consumption
- Added ESC support for APD F/HV/UHV
- Added Hitec protocol
- Fixed Futaba SBUS2
- Fixed Pololu compilation in Arduino IDE
- Improved msrc_gui
- Added support for MS5611

[v0.8](https://github.com/dgatf/msrc/tree/v0.8)

- Added specific drivers for hardware serial and software serial
- Added msrc_gui to create config.h (not all options are avaiable from gui)
- Added Rx protocols: Frsky D, Spektrum XBUS, Spektrum SRXL V5, Flysky Ibus, Futaba SBUS2, Multiplex Sensor Bus, Jeti Ex Bus
- Added support for ATmega328PB, ATmega2560, ATmega32U4, ARM Cortex M0+ and ARM Cortex M4
- Improved accuracy for PWM input (rpm) measurement
- Added ESCs support for: Kontronik Kosmic/Kolibri/JivePro
- Improved current calculation for HW V4/V5

[v0.7](https://github.com/dgatf/msrc/tree/v0.7)

- Added analog airspeed sensor (MPXV7002)
- Fixed Castle Link bug
- Removed BMP180
- Fixed flickering in color lcd displays

[v0.6](https://github.com/dgatf/msrc/tree/v0.6)

- Added GPS serial (BN220, BN880, NEO-M8N,...)

[v0.5](https://github.com/dgatf/msrc/tree/v0.5)

- Added Castle Link Live protocol
- Hobbywing V4/V5. Improved transformations for voltage and current depending on the model (thanks to Commodore8888)

[v0.4.1](https://github.com/dgatf/msrc/tree/v0.4.1)

- Fix Horus display

[v0.4](https://github.com/dgatf/msrc/tree/v0.4)

- Changed R3 resistor to 3.3k
- Support for [change_id_frsky](https://github.com/dgatf/change_id_frsky) to change the sensor id
- Support for I2C sensors 
- Improved code quality and performance
- [Smartport_library](https://github.com/dgatf/smartport) improved performance and abstract from the smartport protocol

[v0.3.1](https://github.com/dgatf/msrc/tree/v0.3.1)

- Added cell voltage average (HW V4/V5, VFAS sensor)
- Applied correct transformation for esc voltage, current and temperature (HW V4/V5)
- Changed averaging type from SMA to EMA
- Added esc protocol NONE
- Smartport protocol. Minor improvements

[v0.3](https://github.com/dgatf/msrc/tree/v0.3)

- Esc current sensor (EscA) added (HW V4/V5, 80A or higher)
- Averaging telemetry added
- Voltage2 sensor changed from A3 to A4
- Ntc2 sensor changed from Tmp1 to Tmp2
- Averaging governor added
- Refresh rate and averaging added to lua config script


## 11. Support

For issues use github:

[Issues](https://github.com/dgatf/msrc/issues)

For questions use the forums:

[RCGroups](https://www.rcgroups.com/forums/showthread.php?4088405-DIY-MSRC-Multi-sensor-telemetry-for-several-Rx-protocols#post48830585) 

[Helifreak](https://www.helifreak.com/showthread.php?t=908371)

[Openrcforums](https://www.openrcforums.com/forum/viewtopic.php?f=84&t=11911)

If you want to add support for new receiver protocol or new sensor, ask for it
