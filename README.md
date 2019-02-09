# ESC RPM telemetry to Smartport

The Hobbywing ESC Platinum V3 has a digital output for the RPM but needs to be converted to Smartport protocol (FrSky). To send the RPM over the smartport I've made a converter with an Arduino Nano which also sends the battery voltage

## ESC telemetry

Either RPM digital out or PWM signal can be used. The signal is sent to Smartport as RPM sensor

- Digital signal does not need to be calibrated but only Hobbywing protocol is implemented (other protocols can be implemented)
- PWM signal can be from any ESC which has this output. The RPM value needs to be calibrated. To use the PWM signal from the ESC comment the sketch line #define ESC_DIGITAL

## Battery voltage

The battery voltage can be measured with a voltage divider (total voltage) or individual cells with a TL084 

- Voltage divider needs to be calibrated in Opentx with multiplier in VFAT sensor
- Individual cells can measured with an op amp like TL084. Then uncomment line #define BATT_SENSOR_CELLS 

If voltage measurement is not needed comment line #define BATT_SENSOR_VOLT

## Voltage divider circuit

Metal resistors are recommended as gives more accurate readings and 0.1W or more
Arduino can read up to 5V and is optimized for readings inputs with signal impedance of 10K

To select R values apply formulas: 

Vo=Vi*R2/(R1+R2)<5V
Z=1/((1/R1)+(1/R2))<10K

![Image](./images/Resistive_divider.png?raw=true)

For 3S battery you can choose :

- R1 22K
- R2 12K

For more than 3S change R values or you may burn the Arduino!

## Wiring (with voltage divider):

- ESC Vcc to Arduino Vcc
- ESC Gnd to Arduino Gnd
- ESC Data to Arduino 2
- Receiver smartport to Arduino 8
- Voltage divider + to A4
- Voltage divider - to GND

![Image](./images/nano1.jpg?raw=true)
![Image](./images/nano2.jpg?raw=true)

# [Youtube video](https://youtu.be/q-e1SoEPNao)

## Adjusting RPM value

If using digital RPM and leaving POLES 1 in the code, adjust RPM sensor in Opentx:

- Blades/poles: number of pair of poles * main gear teeth
- Multiplies: pinion gear teeth

If using PWM, signal needs to be calibrated with a tachometer

Altough max head speed can be estimated with the formula: 

Head speed =(pack voltage * motor kv)/(main gear/pinion)
Then apply an efficiency factor of 90%

It has to be measured without blades at full throttle (disable governor). Then adjust the value in Opentx with the divisor Poles/blades in RPM sensor

## Flash to Arduino

If using Arduino IDE create a folder 'rpm_volt_sensor' and copy the files from the folder in src. Select Arduino Nano and flash it
