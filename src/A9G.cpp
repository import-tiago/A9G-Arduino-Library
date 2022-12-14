#include "A9G.h"

// GPRS ----------------------------------------------
char gprs_uart_buffer[GPRS_UART_BUFFER_LEN];
unsigned int gprs_uart_buffer_index;
HardwareSerial* cmds_port;

// GPS ----------------------------------------------
char gps_uart_buffer[GPRS_UART_BUFFER_LEN];
unsigned int gps_uart_buffer_index;
HardwareSerial* gps_port;
double _lat, _lng, _satellites_qty;
TinyGPSPlus NMEA;
//---------------------------------------------------

bool A9G_Controller::turn_on(uint32_t timeout_secs) {

	timeout_secs *= 1000;

	if (_on_off_pin >= 0) {
		digitalWrite(_on_off_pin, HIGH);
		delay(10);
	}
	digitalWrite(_reset_pin, HIGH);
	delay(100);
	digitalWrite(_init_pin, LOW);
	delay(100);
	digitalWrite(_reset_pin, LOW);
	delay(100);

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	uint32_t t0 = millis();
	while (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"READY", 5) < 0) {

		serialEventRun();

		if (millis() - t0 >= timeout_secs)
			return false;
	}
	return true;
}

void A9G_Controller::turn_off() {

	if (_on_off_pin >= 0)
		digitalWrite(_on_off_pin, LOW);

	digitalWrite(_init_pin, LOW);
	digitalWrite(_reset_pin, LOW);
	delay(500);
}

bool A9G_Controller::restart(uint32_t timeout_secs) {

	timeout_secs *= 1000;

	if (_on_off_pin >= 0) {
		digitalWrite(_on_off_pin, LOW);
		delay(500);
		digitalWrite(_on_off_pin, HIGH);
		delay(10);
	}
	digitalWrite(_reset_pin, HIGH);
	delay(100);
	digitalWrite(_init_pin, LOW);
	delay(100);
	digitalWrite(_reset_pin, LOW);
	delay(100);

	uint32_t t0 = millis();
	while (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"READY", 5) < 0) {
		if (millis() - t0 >= timeout_secs)
			return false;
	}
	return true;
}

/**
 * @brief It updates data from internal peripherals, reading the GPS and GPRS serial port.
 *
 */
void A9G_Controller::loop() {
	serialEventRun();
}

/**
 * @brief allows turning on or off the echo of AT commands
 *
 * @param mode
 * @return true
 * @return false
 */
bool A9G_Controller::echo(bool mode) {

	char cmd[5];

	if (mode)
		sprintf(cmd, "ATE1");
	else
		sprintf(cmd, "ATE0");

	return Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 2000);
}

int A9G_Controller::memsearch(char* source, int sourceLen, char* target, int targetLen) {
	int i, j;
	sourceLen -= targetLen;

	for (i = 0; i < sourceLen; i++) {
		for (j = 0; j < targetLen; j++) {
			if (source[i + j] != target[j])
				break;
		}

		if (j == targetLen) {

			return i;
		}
	}
	return -1;
}

/* #include <stdio.h>
#include <stdlib.h>
#include <string.h>   */

char* A9G_Controller::replaceWord(const char* s, const char* oldW, const char* newW) {
	char* result;
	int i, cnt = 0;
	int newWlen = strlen(newW);
	int oldWlen = strlen(oldW);

	// Counting the number of times old word
	// occur in the string
	for (i = 0; s[i] != '\0'; i++) {
		if (strstr(&s[i], oldW) == &s[i]) {
			cnt++;
			// Jumping to index after the old word.
			i += oldWlen - 1;
		}
	}

	// Making new string of enough length
	result = (char*)malloc(i + cnt * (newWlen - oldWlen) + 1);

	i = 0;

	while (*s) {
		// compare the substring with the result
		if (strstr(s, oldW) == s) {
			strcpy(&result[i], newW);
			i += newWlen;
			s += oldWlen;
		}
		else
			result[i++] = *s++;
	}

	result[i] = '\0';

	return result;
}

bool A9G_Controller::Send_and_Wait_Response(HardwareSerial& serial, char* cmd, char* expected_answer, unsigned long timeout) {

	bool ret = false;

	memset(gprs_uart_buffer, '\0', gprs_uart_buffer_index);
	gprs_uart_buffer_index = 0;
	serial.println(cmd);
	serial.flush();

	unsigned long t0 = millis();
	while (millis() - t0 < timeout) {

		serialEventRun();

		if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, expected_answer, strlen(expected_answer)) >= 0) {
			ret = true;
			break;
		}
		if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) >= 0) {
			ret = false;
			break;
		}
	}

	return ret;
}

char* GPRS_Controller::get_imei() {

	static char IMEI[16];
	int beginIndex, endIndex;

	if (Send_and_Wait_Response(*cmds_port, (char*)"AT+GSN", (char*)"OK", 5000)) {

		beginIndex = memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+GSN\r\n\r\n", 10);
		endIndex = memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"\r\n\r\nOK", 6);

		if (beginIndex > -1 && endIndex > -1) {
			strncpy(IMEI, gprs_uart_buffer + beginIndex + 10, endIndex - (beginIndex + 10));
			IMEI[15] = '\0';
			return IMEI;
		}
	}

	return 0;
}

uint16_t GPRS_Controller::signal_level(uint8_t output_type) {

	static char RSSI[5];
	int beginIndex, endIndex;

	memset(RSSI, '\0', 5);

	if (Send_and_Wait_Response(*cmds_port, (char*)"AT+CSQ", (char*)"OK", 5000)) {

		beginIndex = memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"+CSQ: ", 6);
		endIndex = memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)",", 1);

		if (beginIndex > -1 && endIndex > -1) {
			strncpy(RSSI, gprs_uart_buffer + beginIndex + 6, endIndex - (beginIndex + 6));

			if (output_type == PERCENTAGE_LEVEL)
				return (uint16_t)map(atoi(RSSI), 0, 31, 0, 100);
			else if (output_type == RSSI_LEVEL)
				return atoi(RSSI);
		}
	}

	return 0;
}

bool GPRS_Controller::cellular_network_connect(char* network_apn) {
	unsigned int step = 0;
	char cmd[150];
	unsigned int error_count = 1;

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	do {
		switch (step) {

			case 0: { // Set base station location
				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+CGATT=1", (char*)"OK", 5000)) {

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+CGATT=1", 10) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {
							step++;
							error_count = 0;
						}
						else
							error_count++;
					}
				}

				break;
			}

			case 1: { // checks if location is from base station
				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+CREG?", (char*)"+CREG: 1,1\r\n\r\nOK", 15000))
					step++;

				break;
			}

			case 2: { // Set PDP parameter (APN registering)

				sprintf(cmd, "AT+CGDCONT=1,\"IP\",\"%s\"", network_apn);

				if (Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 5000)) {

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+CGDCONT=1", 12) >= 0) {
						step++;
					}
					error_count++;
				}
				else
					error_count++;
				break;
			}

			case 3: { // Activate PDP (receives an IP address and allows access to internet)

				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+CGACT=1,1", (char*)"OK", 30000)) {

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+CGACT=1,1", 12) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {
							step++;

							error_count = 0;
						}
						else
							error_count++;
					}
				}
				else
					error_count++;

				break;
			}
		}

		if (error_count >= 2)
			return false;

	} while (step <= 3);

	return true;
}

bool GPRS_Controller::mqtt_connect_broker(char* host_address, unsigned int host_port, char* host_user, char* host_password, char* client_id, unsigned int client_time_alive) {
	//bool ret = false;
	unsigned int step = 0;
	char cmd[150];
	unsigned int error_count = 1;

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	do {
		switch (step) {

			case 0: { // Ensure broker is disconnected

				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+MQTTDISCONN", (char*)"OK", 5000)) {

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+MQTTDISCONN", 14) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {
							step++;
							error_count = 0;
						}
						else
							error_count++;
					}
				}
				break;
			}

			case 1: { // Try connection to mqtt borker

				sprintf(cmd, "AT+MQTTCONN=\"%s\",%d,\"%s\",%d,1,\"%s\",\"%s\"", host_address, host_port, client_id, client_time_alive, host_user, host_password);

				if (Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 40000)) {
					step++;
					error_count = 0;
				}
				else
					error_count++;

				break;
			}
		} // switch ends

		if (error_count >= 3)
			return false;

	} while (step <= 1);

	return true;
}

bool GPRS_Controller::mqtt_publish(char* topic, char* payload, uint8_t qos) {

	char* result = NULL;
	bool quoted_payload = false;

	if (strstr(payload, "\"") != NULL) {
		quoted_payload = true;
		result = replaceWord(payload, "\"", "\x5c\x32\x32");
	}

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	uint8_t duplicate_flag = 0, retain = 0;

	char cmd[512];

	if (!quoted_payload)
		sprintf(cmd, "AT+MQTTPUB=\"%s\",\"%s\",%u,%u,%u", topic, payload, duplicate_flag, qos, retain);
	else {
		sprintf(cmd, "AT+MQTTPUB=\"%s\",\"%s\",%u,%u,%u", topic, result, duplicate_flag, qos, retain);
		free(result);
	}

	return Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 30000);
}

bool GPRS_Controller::mqtt_subscribe(char* topic, uint8_t qos, voidFuncPtr CallBackFuncitonPointer) {

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	char cmd[100];

	_CallBackFuncitonPointer = CallBackFuncitonPointer;

	sprintf(cmd, "AT+MQTTSUB=\"%s\",1,%u", topic, qos);

	return Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 30000);
}

bool GPRS_Controller::mqtt_unsubscribe(char* topic, uint8_t qos) {

	char cmd[100];

	sprintf(cmd, "AT+MQTTSUB=\"%s\",0,%u", topic, qos);

	return Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 30000);
}

void GPRS_Controller::mqtt_loop() {

	if (_CallBackFuncitonPointer) {

		int i = memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"+MQTTPUBLISH:", 13);


		if (i >= 0) {

			static uint32_t t0;
			static bool update = true;

			if (update) {
				t0 = millis();
				update = false;
			}

			if ((millis() - t0) > 100) {
				update = true;


				for (int y = 0; y < (i + 13); y++)
					gprs_uart_buffer[y] = '.';

				char topic[100];
				int start = -1, end = -1;


				for (int y = 0; y < GPRS_UART_BUFFER_LEN; y++) {
					if (gprs_uart_buffer[y] == ',') {
						start = y;

						for (int z = start + 1; z < GPRS_UART_BUFFER_LEN; z++) {
							if (gprs_uart_buffer[z] == ',') {
								end = z;

								y = GPRS_UART_BUFFER_LEN;
								z = GPRS_UART_BUFFER_LEN;
							}
						}
					}
				}

				int z = 0;
				for (int y = start + 1; y < end; y++, z++)
					topic[z] = gprs_uart_buffer[y];

				topic[z] = '\0';
				int topic_len = strlen(topic);

				for (int y = i; y < end; y++)
					gprs_uart_buffer[y] = '.';

				char payload_len_arr[16];
				int payload_len = 0;

				for (int y = 0; y < GPRS_UART_BUFFER_LEN; y++) {
					if (gprs_uart_buffer[y] == ',') {
						start = y;

						for (int z = start + 1; z < GPRS_UART_BUFFER_LEN; z++) {
							if (gprs_uart_buffer[z] == ',') {
								end = z;

								y = GPRS_UART_BUFFER_LEN;
								z = GPRS_UART_BUFFER_LEN;
							}
						}
					}
				}

				z = 0;
				for (int y = start + 1; y < end; y++, z++)
					payload_len_arr[z] = gprs_uart_buffer[y];

				payload_len_arr[z] = '\0';

				payload_len = atoi(payload_len_arr);

				char payload[200];

				z = 0;
				for (int y = end + 1; z < payload_len; y++, z++)
					payload[z] = gprs_uart_buffer[y];

				payload[z] = '\0';

				_CallBackFuncitonPointer(topic, payload, payload_len);
			}
		}
	}
}

bool GPS_Controller::enable(uint8_t nmea_refresh_time_secs) {

	unsigned int step = 0;
	unsigned int error_count = 0;
	bool state = DISABLED;

	memset(gprs_uart_buffer, '\0', GPRS_UART_BUFFER_LEN);
	gprs_uart_buffer_index = 0;

	do {
		switch (step) {
			case 0: {
				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+GPS=1", (char*)"OK", 2000)) { // turn on GPS peripheral on A9G

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+GPS=1", 8) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {

							step++;
							state = true;
							error_count = 0;
						}
						else
							error_count++;
					}
					else
						error_count++;
				}
				break;
			}

			case 1: {

				char cmd[16] = { '\0' };
				sprintf(cmd, "AT+GPSRD=%u", nmea_refresh_time_secs);
				int cmd_len = strlen(cmd);

				if (Send_and_Wait_Response(*cmds_port, cmd, (char*)"OK", 2000)) { // Print NMEA every N seconds

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, cmd, cmd_len) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {

							step++;
							state = true;
							error_count = 0;
						}
						else
							error_count++;
					}
					else
						error_count++;
				}
				break;
			}

			case 2: {
				if (Send_and_Wait_Response(*cmds_port, (char*)"AT+GPSUPGRADE=1", (char*)"OK", 2000)) { // Enable GPS UART at 9600 baud (default baud)

					if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"AT+GPSUPGRADE=1", 15) >= 0) {
						if (memsearch(gprs_uart_buffer, GPRS_UART_BUFFER_LEN, (char*)"ERROR", 5) < 0) {

							step++;
							state = true;
							error_count = 0;
						}
						else
							error_count++;
					}
					else
						error_count++;
				}
				break;
			}
		}

		if (error_count >= 2) {
			return false;
		}

	} while (step <= 2);

	return state;
}

double GPS_Controller::location(uint8_t parameter) {

	switch (parameter) {

		case LAT:
			return _lat;

		case LNG:
			return _lng;

		case QTY:
			return _satellites_qty;
	}

	return 0;
}

void update_gps_data() {

	if (NMEA.location.isUpdated() && NMEA.location.isValid()) {
		_lng = NMEA.location.lng();
		_lat = NMEA.location.lat();
		_satellites_qty = (double)NMEA.satellites.value();
	}
}

void serialEvent() {

	if (cmds_port == &Serial) {
		gprs_uart_buffer[gprs_uart_buffer_index] = cmds_port->read();

		if (++gprs_uart_buffer_index >= GPRS_UART_BUFFER_LEN)
			gprs_uart_buffer_index = 0;
	}

	else if (gps_port == &Serial) {
		NMEA.encode(gps_port->read());
		update_gps_data();
	}
}

void serialEvent1() {

	if (cmds_port == &Serial1) {
		gprs_uart_buffer[gprs_uart_buffer_index] = cmds_port->read();

	#ifdef DEBUG_SERIAL
		Serial.print(gprs_uart_buffer[gprs_uart_buffer_index]);
	#endif

		if (++gprs_uart_buffer_index >= GPRS_UART_BUFFER_LEN)
			gprs_uart_buffer_index = 0;
	}

	else if (gps_port == &Serial1) {
		NMEA.encode(gps_port->read());
		update_gps_data();
	}
}

void serialEvent2() {

	if (NMEA.location.isUpdated() && NMEA.location.isValid()) {
		_lng = NMEA.location.lng();
		_lat = NMEA.location.lat();
		_satellites_qty = NMEA.satellites.value();
	}

	if (cmds_port == &Serial2) {
		gprs_uart_buffer[gprs_uart_buffer_index] = cmds_port->read();

	#ifdef DEBUG_SERIAL
		Serial.print(gprs_uart_buffer[gprs_uart_buffer_index]);
	#endif

		if (++gprs_uart_buffer_index >= GPRS_UART_BUFFER_LEN)
			gprs_uart_buffer_index = 0;
	}

	else if (gps_port == &Serial2) {
		NMEA.encode(gps_port->read());
		update_gps_data();
	}
}

