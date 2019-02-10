/*
 * rpm_volt_sensor v0.3
 *    09/02/2019
 *
 * Arduino sketch to send to Frsky SmartPort ESC RPM and battery voltage:
 *
 * - Hobywing digitaL ESC RPM or any ESC with RPM PWM out
 * - Battery voltage with voltage divider or lipo cells
 *
 * Adjusting POLES in the sketch is optional and this can be adjusted in the
 * Opentx. If you leave POLES 1:
 *
 * - Blades/poles: number of pair of poles * main gear teeth
 * - Multiplies: pinion gear teeth
 *
 * Pinout
 * ------
 *
 * - ESC Vcc to Arduino Vcc
 * - ESC Gnd to Arduino Gnd
 * - Receiver smartport to Arduino 8
 * - ESC Data to Arduino 2
 * - Voltage divider + to A4
 * - Voltage divider - to GND
 *
 * author: Daniel Gorbea <danielgorbea@hotmail.com>
 *
 */

#define POLES 1
#define PIN_SMARTPORT 8
#define PIN_ESC 2
#define PIN_CELL1 A1
#define PIN_CELL2 A2
#define PIN_CELL3 A3
#define PIN_BATT A4
#define CELLS 3
#define ESC_DIGITAL
//#define BATT_SENSOR_CELLS
#define BATT_SENSOR_VOLT
#define RPM_QUEUE_SIZE 20
#define VOLT_QUEUE_SIZE 20

#include <Arduino.h>
#include <SoftwareSerial.h>

void queueInit();
void sendByte(uint8_t c, uint16_t *crcp);
void sendByte(uint8_t c, uint16_t *crcp);
void sendData(uint16_t id, int32_t val);
uint32_t lipoCell(uint8_t id, float val1, float val2);
uint32_t lipoCell(uint8_t id, float val);
void sendVolt(float volt);
void sendCell(float cell1, float cell2, float cell3);
void sendRpm(float rpm);
float escDigitalRead(SoftwareSerial &escSerial);
float escPwmRead();
void readCell(float &cell1, float &cell2, float &cell3);
float readVolt();
void setup();
void loop();

template <typename T>
class Queue {
	class Node {
	public:
		T item;
		Node* next;
		Node() {
			next = NULL;
		}
		~Node() {
			next = NULL;
		}
	};
	Node* head;
	Node* tail;
public:
	Queue() {
		head = NULL;
		tail = NULL;
	}

	~Queue() {
		for (Node* node = head; node != NULL; node = head) {
			head = node->next;
			delete node;
		}
	}

	bool enqueue(T item) {
		Node* node = new Node;
		if (node == NULL) {
			return false;
		}
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
		if (head == NULL) {
			return T();
		}
		Node* node = head;
		head = node->next;
		T item = node->item;
		delete node;
		node = NULL;
		if (head == NULL) {
			tail = NULL;
		}
		return item;
	}

	T front() {
		if (head == NULL) {
			return T();
		}
		T item = head->item;
		return item;
	}
};

Queue<uint32_t> queueRpm;
float avRpm = 0;
SoftwareSerial smartportSerial(PIN_SMARTPORT, PIN_SMARTPORT, true);
#ifdef BATT_SENSOR_VOLT
Queue<float> queueVolt;
float avVolt = 0;
#endif
#ifdef ESC_DIGITAL
SoftwareSerial escSerial(PIN_ESC, PIN_ESC);
#endif
