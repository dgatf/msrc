# ESC telemetry to FrSky Smartport

Send ESC telemetry to Frsky Smartport using an Arduino Pro Mini 168 or 328P (3.3v or 5v)

## ESC telemetry

The input is either ESC serial data or PWM signal

ESC serial protocols implemented:

- Hobbywing Platinum V3: RPM
- Hobbywing Platinum V4, Hobbywing Flyfun V5: RPM, temperature and battery voltage
- PWM signal: RPM

## PWM output

PWM signal generation from ESC serial (for HW V5 which does not have PWM output) for FLB systems

PWM signal properties: logic level 3.3V and duty cycle 17%

## Voltage divider circuit (optional)

Metal resistors are recommended as gives more accurate readings (0.1W or higher)
Arduino can read up to 3.3V/5V and is optimized for signal inputs with 10K impedance

To select R values apply formulas:

*Vo = Vi\*R2/(R1+R2) < 3.3V or 5V*

*Z = 1/((1/R1)+(1/R2)) < 10K*

<p align="center"><img src="./images/Resistive_divider.png" width="200"></p>

For 6S battery (or lower) and Pro Mini 3.3v:

 - R1 68k
 - R2 10k

If more than 6S change R values or you may burn the Arduino!

## Thermistors (optional)

Temperature is measured with NTC thermistors (100k). Adjust thermistor Beta if needed (default is 4190). Sensor output us in Celsius


## OpenTx sensors (sensor Id 10)

- RPM: EscR
- ESC voltage: EscV
- ESC temp: EscT
- Voltage divider: A3
- Thermistor 1: Tmp1
- Thermistor 2 : Tmp1

## Wiring:

 - SmartPort Vcc to Arduino RAW
 - SmartPort Gnd to Arduino Gnd
 - Smartport Signal to Arduino PIN_SMARTPORT_RX (7)
 - Smartport Signal to R3 (4.7k)
 - R3 (4.7k) to Arduino PIN_SMARTPORT_TX (12)
 - If using ESC serial: ESC serial signal to Arduino Rx
 - If using ESC PWM: ESC PWM signal to Arduino PIN_PWM_ESC (8)
 - If PWM output is required (for HobbyWing Flyfun V5): Flybarless PWM signal input to Arduino PIN_PWM_OUT (9)
 - Voltage divider + to PIN_BATT (A1)
 - Voltage divider - to Gnd

<p align="center"><img src="./images/esc_smartport11.png" width="600">
<img src="./images/top.jpg" width="400">
<img src="./images/bottom.jpg" width="400"></p>


## Adjust RPM sensor

- Blades/poles: number of pair of poles * main gear teeth  
- Multiplier: pinion gear teeth

## Adjust A3 sensor (for voltage divider)

Measure the voltage of the battery with a voltmeter and adjust *Ratio* in A3 sensor

## Configuration

The configuration is modified with a lua script (openTx 2.2 or higher)

<p align="center"><img src="./images/escSp.bmp" width="300"></p>

Options:

- ESC protocol. HobbyWing Platinum V3, HobbyWing Platinum V4/Hobbywing Flyfun V5 or PWM signal
- Battery. For voltage divider
- PWM. To generate PWM output from ESC serial  (for obbywing Flyfun V5)

Copy the file escSp.lua to the TELEMETRY folder in the sdcard of the Tx and add ther script to a telemetry screen (*Script->escSp*)

## Flash to Arduino

Using Arduino IDE copy folder *esc_smartport* and open *esc_smartport.ino*. Select board *Arduino Pro or Pro Mini*, processor *ATMega168 or ATMega328P (3.3V 8MHz or 5V 16MHz)* and flash

## [Video](https://youtu.be/Mby2rlmAMlU)
