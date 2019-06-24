/*
 * esc_smartport
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Arduino sketch to send to Frsky SmartPort ESC telemetry:
 *
 * - Hobywing V3
 * - Hobywing V4/V5
 * - RPM PWM signal supported
 * - Battery voltage with voltage divider
 * - PWM output (for HW V5 Flyfun)
 *
 * Adjust RPM sensor in OpenTx:
 *
 * - Blades/pair pof poles: number of pair of poles * main gear teeth
 * - Multiplies: pinion gear teeth
 *
 * Wiring
 * ------
 *
 * - SmartPort Vcc to Arduino RAW
 * - SmartPort Gnd to Arduino Gnd
 * - Smartport Signal to Arduino PIN_SMARTPORT_RX (7)
 * - Smartport Signal to R3 (4.7k)
 * - R3 (4.7k) to Arduino PIN_SMARTPORT_TX (12)
 * - If using ESC serial: ESC serial signal to Arduino Rx
 * - If using ESC PWM: ESC PWM signal to Arduino PIN_PWM_ESC (8)
 * - If PWM output is required (for HobbyWing Flyfun V5): Flybarless PWM signal
 * input to Arduino PIN_PWM_OUT (9)
 * - Voltage divider + to PIN_BATT (A1)
 * - Voltage divider - to Gnd
 *
 */

// Version

#define VERSION_MAJOR 0
#define VERSION_MINOR 3

// Pins

#define PIN_SMARTPORT_RX 7
#define PIN_SMARTPORT_TX 12

// Analog inputs

#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A4

// Telemetry refresh rate in ms

#define REFRESH_RPM 200
#define REFRESH_VOLT 1000
#define REFRESH_TEMP 1000
#define REFRESH_CURR 1000

// Queues sizes

#define QUEUE_RPM 20
#define QUEUE_VOLT 20
#define QUEUE_TEMP 20
#define QUEUE_CURR 20

// Board Vcc

#if F_CPU == 16000000UL
#define BOARD_VCC 5
#else
#define BOARD_VCC 3.3
#endif

// Thermistors (NTC 100k, R1 10k)

#define NTC_R_REF 100000UL
#define NTC_R1 10000
#define NTC_BETA 4190
//#define NTC_A1 3.35E-03
//#define NTC_B1 2.46E-04
//#define NTC_C1 3.41E-06
//#define NTC_D1 1.03E-07

// Config bitmask

// byte 1: config
#define BITMASK_PROTOCOL 0B00000011
#define BITMASK_VOLTAGE1 0B00000100
#define BITMASK_VOLTAGE2 0B00001000
#define BITMASK_CURRENT 0B00010000
#define BITMASK_NTC1 0B00100000
#define BITMASK_NTC2 0B01000000
#define BITMASK_PWM 0B10000000

// byte 2: free

// byte 3: version minor

// byte 4: version major

#define ESCSERIAL_TIMEOUT 3
#define escSerial Serial

// Debug. Uncommnent for debugging
// Disconnect Vcc from the RC model to the Arduino
// Do not connect at the same time Vcc from the model and usb (FTDI)
// Telemetry may not work properly in debug mode
// Connect arduino Rx to FTDI Tx for flashing, then connect arduino Rx to esc

//#define DEBUG

#include "Esc.h"
#include "Smartport.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

template <typename T> class Queue {
  class Node {
  public:
    T item;
    Node *next;
    Node() { next = NULL; }
    ~Node() { next = NULL; }
  };
  Node *head;
  Node *tail;

public:
  Queue() {
    head = NULL;
    tail = NULL;
  }

  ~Queue() {
    for (Node *node = head; node != NULL; node = head) {
      head = node->next;
      delete node;
    }
  }

  bool enqueue(T item) {
    Node *node = new Node;
    if (node == NULL)
      return false;
    node->item = item;
    if (head == NULL) {
      head = node;
      tail = node;
      return true;
    }
    tail->next = node;
    tail = node;
    return true;
  }

  T dequeue() {
    if (head == NULL)
      return T();
    Node *node = head;
    head = node->next;
    T item = node->item;
    delete node;
    node = NULL;
    if (head == NULL)
      tail = NULL;
    return item;
  }

  void init(T item, uint8_t size) {
    for (uint8_t i = 0; i < size; i++)
      this->enqueue(item);
  }

  /*void del() {
    T item = this->dequeue();
    while (item != NULL) {
      item = this->dequeue();
    }
  }*/

  void del(uint8_t size) {
    for (uint8_t i = 0; i < size; i++)
      this->dequeue();
  }

};

// Default config

struct Config {
  uint8_t protocol = PROTOCOL_HW_V3;
  bool voltage1 = false;
  bool voltage2 = false;
  bool current = false;
  bool ntc1 = false;
  bool ntc2 = false;
  bool pwmOut = false;
};

struct Telemetry {
  float *escRpmConsP = NULL;
  float *escPowerP = NULL;
  float *voltageP = NULL;
  float *currentP = NULL;
  float *temp1P = NULL;
  float *temp2P = NULL;
  float *voltageAnalog1P = NULL;
  float *voltageAnalog2P = NULL;
  float *currentAnalogP = NULL;
  float *ntc1P = NULL;
  float *ntc2P = NULL;
  Queue<float> escRpmConsQP;
  Queue<float> escPowerQP;
  Queue<float> voltageQP;
  Queue<float> currentQP;
  Queue<float> temp1QP;
  Queue<float> temp2QP;
  Queue<float> voltageAnalog1QP;
  Queue<float> voltageAnalog2QP;
  Queue<float> currentAnalogQP;
  Queue<float> ntc1QP;
  Queue<float> ntc2QP;
};

void readConfig();
void writeConfig();
void initConfig();
float readVoltageAnalog(uint8_t pin);
float readNtc(uint8_t pin);
void setup();
void loop();

Config config;
Telemetry telemetry;

SoftwareSerial smartportSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX, true);
Smartport smartport(smartportSerial);
Esc esc(escSerial);
