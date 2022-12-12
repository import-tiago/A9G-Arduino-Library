#include <Arduino.h>
#include "A9G.h"

#define A9G_RESET_PIN 26
#define A9G_INIT_PIN 4

#define NETWORK_APN "xpto.com.br"

#define MQTT_BROKER_ADDR "mqtt.xpto.io"
#define MQTT_BROKER_PORT 1883
#define MQTT_BROKER_AUTH_USER "xpto"
#define MQTT_BROKER_AUTH_PASSWORD "xpto"

#define MQTT_CLIENT_ID "A9G"
#define MQTT_CLIENT_KEEPALIVE_SECS 120

#define MQTT_PUB_TOPIC "pub"
#define MQTT_PUB_PAYLOAD "Hello World!"
#define MQTT_PUB_QOS 1

#define MQTT_SUB_TOPIC "sub"
#define MQTT_SUB_QOS 1

#define GPS_REFRESH_INTERVAL_SECS 5

A9G_Controller A9G(A9G_RESET_PIN, A9G_INIT_PIN);
GPS_Controller GPS(Serial1);
GPRS_Controller GPRS(Serial2);

void mqtt_callback(char* topic, char* payload, int length) {
	Serial.printf("\r\nFrom MQTT subscription: topic: %s, payload: %s, length: %d\r\n\r\n", topic, payload, length);
}

void setup() {

	Serial.begin(115200);

	while (!A9G.turn_on(120)) {
		Serial.println("\r\nA9G init fail. Retrying...\r\n");
		A9G.turn_off();
	}

	A9G.echo(true);

	GPRS.cellular_network_connect(NETWORK_APN);

	GPS.enable(GPS_REFRESH_INTERVAL_SECS);

	GPRS.mqtt_connect_broker(MQTT_BROKER_ADDR, MQTT_BROKER_PORT, MQTT_BROKER_AUTH_USER, MQTT_BROKER_AUTH_PASSWORD, MQTT_CLIENT_ID, MQTT_CLIENT_KEEPALIVE_SECS);

	GPRS.mqtt_subscribe(MQTT_SUB_TOPIC, MQTT_SUB_QOS, mqtt_callback);

	GPRS.mqtt_publish(MQTT_PUB_TOPIC, MQTT_PUB_PAYLOAD, MQTT_PUB_QOS);
}

void loop() {

	serialEventRun();
	GPRS.mqtt_loop();

	static uint32_t t0 = millis();

	if (millis() - t0 > 1000) {
		t0 = millis();
		
		/*
		
		NOTE:
		- Send JSON through AT commands is not possible becouse the double quotes ["].
		- That are unfortunately interpreted according to AT commands ETSI specification as the beginning of a string parameter.
		- So, is impossible send a JSON string as a parameter.
		- Use simple quotes ['] could be a option, but will require the server to replace it with double quotes.
		
		*/

		static char gpsData[100];
		sprintf(gpsData, "{\'location\':{\'lat\':%.8f,\'lng\':%.8f,\'qty\':%.0f}}", GPS.location(LAT), GPS.location(LNG), GPS.location(QTY));
		Serial.println(gpsData);
	}
}