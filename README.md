# A9G Arduino Library

Library to use A9G with AT commands in Arduino framework-based systems.

<p align="center"><img src="https://github.com/TiagoPaulaSilva/A9G-Arduino-Library/blob/main/assets/A9G_Module_Preview.png" ></p>

[A9G](http://www.ai-thinker.com/pro_view-28.html) is a quad-band module from [Ai-Thinker](http://www.ai-thinker.com/) based on [RDA8955](https://w.electrodragon.com/w/images/9/97/RDA8955L_Datasheet_v1.0.0.pdf) SoC - [GSM/GPRS] + [GPS/BDS] (850/900/1800/1900MHz).

Official A9G repository: [Ai-Thinker GPRS C SDK](https://github.com/Ai-Thinker-Open/GPRS_C_SDK).

This library configures the A9G's GPS data output using the [NMEA](https://en.wikipedia.org/wiki/NMEA_0183) data specification. So to decode this [NMEA](https://en.wikipedia.org/wiki/NMEA_0183) packet this library incorporated [Mikal Hart](https://github.com/mikalhart)'s amazing [TinyGPSPlus ](https://github.com/mikalhart/TinyGPSPlus) library.


[Minimal example](https://github.com/TiagoPaulaSilva/A9G-Arduino-Library/blob/main/examples/simple/main.cpp) code snippet: 
```c++
#include "A9G.h"

A9G_Controller A9G(A9G_RESET_PIN, A9G_INIT_PIN);
GPS_Controller GPS(Serial1);
GPRS_Controller GPRS(Serial2);

void mqtt_callback(char* topic, char* payload, int length) {
	Serial.printf("\r\nFrom MQTT subscription: topic: %s, payload: %s, length: %d\r\n\r\n", topic, payload, length);
	// GPRS.mqtt_unsubscribe(MQTT_SUB_TOPIC, MQTT_SUB_QOS);
}

void setup() {

	Serial.begin(115200);

	while (!A9G.turn_on(MODEM_INIT_TIMEOUT_SECS)) {
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

	A9G.loop();
	GPRS.mqtt_loop();

	static uint32_t t0 = millis();

	if (millis() - t0 > 10000) {
		t0 = millis();

		static char payload[100];
		sprintf(payload, "{\'location\':{\'lat\':%.8f,\'lng\':%.8f,\'qty\':%.0f}}", GPS.location(LAT), GPS.location(LNG), GPS.location(QTY));

		/*
			NOTE:
			- Send JSON through AT commands is not possible because the double quotes ["].
			- That are unfortunately interpreted (according to AT commands ETSI specification) as the beginning of a string parameter.
			- So, is impossible send a JSON string as a parameter.
			- Use simple quotes ['] could be a option, but will require the server to replace it with double quotes.
		*/

		GPRS.mqtt_publish((char*)"GPS", payload, MQTT_PUB_QOS);
	}
}
```

# Minimal Hardware

<p align="center"><a href="https://github.com/TiagoPaulaSilva/A9G-Arduino-Library/blob/main/assets/A9G%20Minimal%20Hardware/A9G%20Minimal%20Hardware.pdf"><img src="https://github.com/TiagoPaulaSilva/A9G-Arduino-Library/blob/main/assets/schematic_preview.png"  title="Circuit Preview" alt="PDF Download"></a></p>
