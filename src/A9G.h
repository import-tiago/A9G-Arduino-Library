/**
 * @file A9G.h
 * @author Tiago Silva (tiagodepaulasilva@gmail.com)
 * @brief A9G is a  quad-band module based on RDA8955 SoC - [GSM/GPRS] + [GPS/BDS] (850/900/1800/1900MHz)
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef A9G_H
#define A9G_H

#include <Arduino.h>
#include <decoder/TinyGPS++.h>

#define DEBUG_SERIAL

 // GPRS ----------------------------------------------
#define GPRS_UART_BUFFER_LEN 200
#define GPRS_UART_BAUDRATE 115200
extern char gprs_uart_buffer[GPRS_UART_BUFFER_LEN];
extern unsigned int gprs_uart_buffer_index;
extern HardwareSerial* cmds_port;
// ---------------------------------------------------

// GPS ----------------------------------------------
#define GPS_UART_BUFFER_LEN 200
#define GPS_UART_BAUDRATE 9600
#define LAT 0
#define LNG 1
#define QTY 2
extern char gps_uart_buffer[GPRS_UART_BUFFER_LEN];
extern unsigned int gps_uart_buffer_index;
extern HardwareSerial* gps_port;
extern TinyGPSPlus NMEA;
//---------------------------------------------------


// MQTT ----------------------------------------------
typedef void (*voidFuncPtr)(char* topic, char* payload, int length);
static voidFuncPtr  _CallBackFuncitonPointer;
// ---------------------------------------------------

void serialEvent();
void serialEvent1();
void serialEvent2();

class A9G_Controller {
private:
	int8_t _on_off_pin = -1, _init_pin, _reset_pin;

public:

	A9G_Controller(uint8_t on_off_pin, uint8_t reset_pin, uint8_t init_pin): _on_off_pin(on_off_pin), _reset_pin(reset_pin), _init_pin(init_pin) {
		pinMode(on_off_pin, OUTPUT);
		pinMode(reset_pin, OUTPUT);
		pinMode(init_pin, OUTPUT);
	}

	A9G_Controller(uint8_t reset_pin, uint8_t init_pin): _reset_pin(reset_pin), _init_pin(init_pin) {
		pinMode(reset_pin, OUTPUT);
		pinMode(init_pin, OUTPUT);
	}

	bool turn_on(uint32_t timeout_secs);
	void turn_off();
	bool restart(uint32_t timeout_secs);
	bool echo(bool mode);
	int memsearch(char* source, int sourceLen, char* target, int targetLen);
	bool Send_and_Wait_Response(HardwareSerial& serial, char* command, char* expected_answer, unsigned long timeout);
};


class GPRS_Controller: A9G_Controller {
private:
	int8_t _rx_pin = -1, _tx_pin = -1;

public:

	GPRS_Controller(HardwareSerial& serial): A9G_Controller(0, 0, 0) {
		cmds_port = &serial;
		cmds_port->begin(GPRS_UART_BAUDRATE);
	}
	GPRS_Controller(HardwareSerial& serial, uint8_t rx_pin, uint8_t tx_pin): A9G_Controller(0, 0, 0), _rx_pin(rx_pin), _tx_pin(tx_pin) {
		cmds_port = &serial;
		cmds_port->begin(GPRS_UART_BAUDRATE, SERIAL_8N1, _rx_pin, _tx_pin);
	}

	char* get_imei();
	bool cellular_network_connect(char* network_apn);
	bool mqtt_connect_broker(char* host_address, unsigned int host_port, char* host_user, char* host_password, char* client_id, unsigned int client_time_alive);
	bool mqtt_publish(char* topic, char* payload, uint8_t qos);
	bool mqtt_subscribe(char* topic, uint8_t qos, voidFuncPtr CallBackFuncitonPointer);
	bool mqtt_unsubscribe(char* topic, uint8_t qos);
	void mqtt_loop();
};


class GPS_Controller: A9G_Controller {
private:
	uint8_t _rx_pin = -1, _tx_pin = -1;

public:

	GPS_Controller(HardwareSerial& serial): A9G_Controller(0, 0, 0) {
		gps_port = &serial;
		gps_port->begin(GPS_UART_BAUDRATE);
	}
	GPS_Controller(HardwareSerial& serial, uint8_t rx_pin, uint8_t tx_pin): A9G_Controller(0, 0, 0), _rx_pin(rx_pin), _tx_pin(tx_pin) {
		gps_port = &serial;
		gps_port->begin(GPS_UART_BAUDRATE, SERIAL_8N1, _rx_pin, _tx_pin);
	}

	bool enable(uint8_t nmea_refresh_time_secs);
	double location(uint8_t parameter);
};

#endif // A9G_H