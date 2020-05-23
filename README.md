# MSRC - Multi Sensor for RC - FrSky Smartport

This is a DIY project to send multiple sensors telemetry to Frsky Smartport using an Arduino Pro Mini 328P (3.3v or 5v)

## Telemetry

The following types of sensors can be connected:

- ESC
  - ESCs with serial telemetry (Hobbywing V3/V4/V5)
  - ESC with PWM signal or phase sensor
  - ESC Castle Link
- I2C sensors
- Analog sensors

### ESC

Some ESC have a serial port for telemetry output. This can be decoded connecting the ESC to the UART available in the Pro Mini. The following ESC serial protocols are implemented:

- Hobbywing Platinum V3: RPM
- Hobbywing Platinum V4, Hobbywing Flyfun V5: RPM, temperature (Mosfet and BEC), voltage and current

Optionally a PWM signal (PIN 10, 3.3V, 50% duty) can be generated from the RPMs in serial telemetry

<p align="center"><img src="./images/msrc_serial.png" width="600"><br>
  <i>Minimum circuit ESC serial</i><br><br></p>

Some ESC have a PWM signal for motor RPMs, which is equivalent to a phase sensor. Some ESC have both serial and PWM signal, like Hobbywing V4/V5, then PWN signal is not needed for telemetry. Circuit is as follows:

<p align="center"><img src="./images/msrc_pwm.png" width="600"><br>
  <i>Minimum circuit PWM signal</i><br><br></p>

ESC Castle Link protocol goes into the input signal of the ESC. Circuit is as follows:

<p align="center"><img src="./images/msrc_castle.png" width="600"><br>
  <i>Minimum circuit Castle Link</i><br><br></p>

| Model              | RPM         | Throttle    | Motor PWM   | Voltage     | Current   | Voltage BEC | Current BEC | Temperature 1 | Temperature 2 | Ripple Volt |
| ------------------ | :---------: | :---------: | :---------: | :---------: | :-------: | :---------: | :---------: | :-----------: | :-----------: | :---------: |
| Hobbywing V3       | :white_check_mark:         | :white_check_mark:(1)      | :white_check_mark:(1)      |             |           |             |             |               |               |             |
| Hobbywing V4/V5(4) | :white_check_mark:         | :white_check_mark:(1)      | :white_check_mark:(1)      | :white_check_mark:         | :white_check_mark:(2)    |             |             | :white_check_mark: FET       | :white_check_mark: BEC       |             |
| Castle Link        | :white_check_mark:         | :white_check_mark:(1)      | :white_check_mark:(1)      | :white_check_mark:         | :white_check_mark:       | :white_check_mark:(3)      | :white_check_mark:(3)      | :white_check_mark:           |               | :white_check_mark:         |

(1) Available but not forwarded to smartport  
(2) For 80A models and higher  
(3) Not available in all models  
(4) Sensors varies depending on the model and firmware. Update ESC to the latest firmware available. See annex

If voltage is available the  cell voltage average is calculated for 3S,4S,5S,6S,7S,8S,10S and 12S batteries. 10 seconds after power on the number of cells is autodetected. Average cell voltage to be >3.8v for valid a cell count

### Analog sensors

Can be connected the following analog sensors:

- 2 x voltage divider can be added to read the battery voltage (A2, A3)
- 2 x temperature sensors (thermistors) (A0, A1)
- Current sensor (A6)

### I2C sensors

Multiple I2C sensors can be added (A4, A5)

Currently supported:

- Barometer: BMP180, BMP280

<p align="center"><img src="./images/msrc_full.png" width="600"><br>
  <i>Additional sensors</i><br><br></p>

## Flash to Arduino

Using Arduino IDE copy folder *msrc* and open *msrc.ino*. Select board *Arduino Pro or Pro Mini*, processor *ATMega328P (3.3V 8MHz or 5V 16MHz)* and flash

## Configuration

The configuration is modified with a lua script (X7, X9, X-lite and Horus with openTx 2.2 or higher)

<p align="center"><img src="./images/lua_x7.png" height="128">   <img src="./images/lua_x9.png" height="128">   <img src="./images/lua_x10.png" height="200"></p>

Copy the file msrc.lua to the SCRIPTS/TOOLS folder. (if using older openTx 2.2 copy to SCRIPTS folder and execute by long press)

If not using lua script comment *#define CONFIG_LUA* and assign config values in msrc.h

Options:

- ESC protocol. HobbyWing Platinum V3, HobbyWing Platinum V4/Hobbywing Flyfun V5 or PWM signal
- Voltage1. For voltage divider 1
- Voltage2. For voltage divider 2
- Ntc1. Thermistor 1
- Ntc2. Thermistor 2
- Current
- PWM out. Generate a PWM signal from RPM is ESC serial (for Hobbywing Flyfun V5)
- Averaging queue size: 1 to 16
- Refresh rate (ms): 0 to 1600
- I2C (x2). Type and I2C address

## OpenTx sensors

The arduino default sensor id is 10. This can be changed with [change_id_frsky](https://github.com/dgatf/change_id_frsky)

Depending on your configuration some the following sensors will be available in Opentx. After configuration go to sensors screen and update with *Search new sensors*
 
ESC telemetry:

- ESC RPM: Erpm (0x0b60)
- ESC voltage: EscV (0x0b50)
- ESC cell average: VFAS (0x0210)
- ESC current: EscA (0x0b50)
- ESC temp FET (HW) or ESC temp linear (Castle): EscT (0x0b70)
- ESC temp BEC (HW) or ESC temp NTC (Castle): EscT (0x0b71)
- ESC ripple voltage: EscV (0x0b51)
- ESC BEC voltage: EscV (0x0e50)
- ESC BEC current: EscC (0x0e50)

Analog telemetry:

- Voltage 1: A3 (0x0900)
- Voltage 2: A4 (0x0910)
- Thermistor 1: Tmp1 (0x0400)
- Thermistor 2: Tmp2 (0x0410)
- Current: Curr (0x020f)

I2C telemetry:

 - Altitude: Alt (0x0820)
 - Temperature: T1 (0x0401, 0x0402)

Some of the sensors needs to be adusted in openTx

### Adjust RPM sensor (EscR)

- Blades/poles: number of pair of poles * main gear teeth  
- Multiplier: pinion gear teeth

### Adjust voltage sensors (A3, A4)

Measure the voltage of the battery with a voltmeter and adjust *Ratio* in A3, A4 sensor

### Adjust current sensor (Curr)

Adjust sensor ratio: *1000 / output sensitivity (mV/A)*

To get battery consumption add a new sensor:

- Type: Calculated
- Formula: Consumption
- Sensor: Curr

## Images

<p align="center"><img src="./images/top.jpg" width="300">  <img src="./images/bottom.jpg" width="300"></p>

<p align="center"><img src="./images/450_1.jpg" width="300">  <img src="./images/450_2.jpg" width="300"></p>

<p align="center"><img src="./images/450_3.jpg" width="300">  <img src="./images/450_x7.bmp" width="300"><br><i>MSRC on Align 450 connected to Hobbywing V3 Platinum and two thermistors for ESC and motor</i><br></p>



## Video

[Video](https://youtu.be/Mby2rlmAMlU)


## Annex

### ESC protocol specifications Hobbywing

Serial parameters:

- 19200 bps
- 1 packet every 20 ms
- Big endian


#### Hobbywing V3

| Byte  | 1                   | 2                | 3                | 4                | 5             | 6              | 7            | 8            | 9           | 10          |
| ----- | :-----------------: | :--------------: | :--------------: | :--------------: | :-----------: | :------------: | :----------: | :----------: | :---------: | :---------: |
| Value | Package Head (0x9B) | Package Number 1 | Package Number 2 | Package Number 3 | Rx Throttle 1 | Rx Throttle  2 | Output PWM 1 | Output PWM 2 | RPM Cycle 1 | RPM Cycle 2 |

*RPM = 60000000 / RPM Cycle*

rpm, pwm: 0-255 (8bits)

#### Hobbywing V4/V5

| Byte  | 1     | 2     | 3     | 4     | 5     | 6     | 7     | 8     | 9     | 10    | 11    | 12    | 13    | 14    | 15    | 16    | 17    | 18    | 19    |
| ---   | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Value | Package Head (0x9B) | Package Number 1 |	Package Number 2 | Package Number 3	| Rx Throttle 1	| Rx Throttle  2 | Output PWM 1 | Output PWM 2	| RPM 1 | RPM 2	| RPM 3	| Voltage 1 |	Voltage 2	| Current 1	| Current 2	| TempFET 1	| TempFET 2	| Temp 1 |	Temp 2

rpm, pwm: 0-1024 (10bits)

Voltage, current and temperature are raw sensor data. Actual values requires transformation. Depending on the model, sensors are different so  the transformations:

  - Voltage divider. Different for LV and HV models. LV divisor 11. HV divisor 21
  - Current sensor. Different for V4 and V5. V5 seems to be shifted by Vref=0.53V
  - Temperature. NTC resistor is used. So far it is the same for tested models

Before throttle is raised from 0, signature packets are sent between telemetry packets. This is used to identify the hardware and firmware of the ESC

Examples:

| Model\Byte | 1     | 2     | 3     | 4     | 5     | 6     | 7     | 8     | 9     | 10    | 11    | 12    | 13    |
| ---------- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| V4LH80A    | 0x9B  | 0x9B  | 0x03  | 0xE8  | 0x01  | 0x08  | 0x5B  | 0x00  | 0x01  | 0x00  | 0x21  | 0x21  | 0xB9  |
| V5LH130A   | 0x9B  | 0x9B  | 0x03  | 0xE8  | 0x01  | 0x0B  | 0x41  | 0x21  | 0x44  | 0xB9  | 0x21  | 0x21  | 0xB9  |

### ESC protocol specifications Castle Link

For best accuracy RX pulse input is captured with a timer interrupt and ESC pulse output is produced by hardware PWM. Maximum latency is 20ms

See [Castle Link Live](https://dzf8vqv24eqhg.cloudfront.net/userfiles/4671/6540/ckfinder/files/Product%20Manuals/Accessories%20and%20replacement%20parts/castle_link_live_2_0.pdf?dc=201606221536-537)

### Output PWM signal for FBL

For best accuracy PWM signal output for FBL is produced by hardware PWM from serial RPM values. Maximum latency is 40ms

### ADC voltage

To obtain the voltage at the analog pin it is required the ADC bits (b) and the Vref:

<img src="https://latex.codecogs.com/svg.latex?Vo=Vref*\frac{Raw}{2^b}" title="Vo = Vref * Raw / 2^bits" /><br>

### Analog voltage sensors. Voltage divider circuit

Two battery voltages can be measured through the analog pins A2 and A3
Metal resistors are recommended as gives more accurate readings (0.1W or higher)
Arduino can read up to 3.3V/5V and is optimized for signal inputs with 10K impedance

<p align="center"><img src="./images/Resistive_divider.png" width="200"></p>

To select R values apply formulas:

<img src="https://latex.codecogs.com/svg.latex?Vo=Vin\frac{R_2}{R_1+R_2}<3.3Vor5V" title="Vo = Vi\*R2/(R1+R2) < 3.3V or 5V" /><br>

<img src="https://latex.codecogs.com/svg.latex?Z=\frac{1}{\frac{1}{R_1}+\frac{1}{R_2}} < 10K" title="Z = 1/((1/R1)+(1/R2)) < 10K" />


For 6S battery (or lower) and Pro Mini 3.3v:

 - R1 68k
 - R2 10k

If more than 6S change R values or you may burn the Arduino!

### Temperature sensors. Thermistors

Two temperature sensors can be installed through the analog pins A0 and A1
Temperature is measured with NTC thermistors (100k). Adjust thermistor Beta in ntc.h if needed (NTC_BETA, default is 4190). Sensor output in Celsius

<p align="center"><img src="./images/ntc.gif" width="200"></p>

To obtain the thermistor resistance:

<img src="https://latex.codecogs.com/svg.latex?Rt=\frac{Vo*Rs}{(Vin-Vo)}" title="Rt = Vo * Rs / (Vin - voltage))}" />

And temperature with Beta formula:

<img src="https://latex.codecogs.com/svg.latex?T=\frac{1}{\frac{ln\frac{Rt}{Rref}}{\beta}+\frac{1}{Tref}}" title="T = 1/[ln(Rt/Rref)/B+1/Tref]" />

Or with Steinhart and Hart Equation if data is available:

<img src="https://latex.codecogs.com/svg.latex?T=\frac{1}{A+B*ln\frac{Rt}{Rref}+C*ln(\frac{Rt}{Rref})^2+D*ln(\frac{Rt}{Rref})^3}" title="T = 1/[A+Bln(Rt/Rref)+Cln(Rt/Rref)²+Dln(Rt/Rref)³]" />

## Change log

v0.5

- Added Castle Link Live protocol
- Hobbywing V4/V5. Improved transformations for voltage and current (thanks to Comodore8888). Added ESC signatures

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

## Support

For questions, issues or new protocol request (use this [sketch](./sniffer/sniffer.ino)) please post in the forums:

[Helifreak](https://www.helifreak.com/showthread.php?t=835243)

[Openrcforums](https://www.openrcforums.com/forum/viewtopic.php?f=84&t=11911)

Or open an [Issue](https://github.com/dgatf/msrc/issues) in Github


## Acknowledgements

- Commodore8888 (Helifreak)
- MikeJ (Helifreak)
- Atomic Skull (Helifreak)
- McGiverek (Helifreak)