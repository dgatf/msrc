# ESC rpm telemetry to smartport

The hobbywing ESC Platinum V3 has a digital output for the RPM but needs to be converted to the smartport protocol. To send the RPM over the smartport I've made a converter with an Arduino Nano which also sends the battery voltage

Either RPM digital out or PWM signal can be used. Digital signal does not need to be calibrated but is specific for Hobbywing protocol. PWM signal can be from any ESC which has this output but the RPM value needs to be calibrated. To use the PWM signal from the ESC comment the sketch line #define ESC_DIGITAL

The Arduino reads the battery voltage with a voltage divider and send it as VFAT sensor

Also it is possible to read the lipo voltage with and op amp like the TL084 (I made the code but not tested). To use an op amp uncomment the sketch line #define BATT_SENSOR_CELLS and plug the readings from the op amp to A1, A2 and A3 on the Arduino. Readings are send as CELLS sensor

The wiring is (with voltage divider):

- ESC Vcc to Arduino Vcc
- ESC Gnd to Arduino Gnd
- ESC Data to Arduino 2
- Receiver smartport to Arduino 8
- Voltage divider + to A4
- Voltage divider - to GND

For the voltage divider and using up to 3S battery:

- R1 22K
- R2 12K

For more than 3S change R values or you may burn your Arduino!

If you leave POLES 1 in the sketch, then adjust RPM sensor in Opentx:

Blades/poles: number of pair of poles * main gear teeth
Multiplies: pinion gear teeth
