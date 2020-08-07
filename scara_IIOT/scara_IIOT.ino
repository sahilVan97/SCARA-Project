/*
 Name:		scara_IIOT.ino
 Created:	10/3/2019 10:36:53 AM
 Author:	sahil Vanarase
*/
#include <Arduino.h>
#include <WiFiNINA.h>
#include "wiring_private.h"
#include <SPI.h>


const int slaveAPin = 9; // SS (PA18 / D9 on SAMD21)
uint8_t data = 0;  
char _data = data;

#define MOSI 11
#define MISO 12
#define SCK  13
#define SS   9

char ssid[] = "ASK4 Wireless";        // your network SSID (name)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
// ThingSpeak Settings
//Channel ID: 900366
char thingSpeakAddress[] = "api.thingspeak.com";
const String APIKey = "73SELUB1S6SF49WO";             // channel's Write API Key
const String readAPIKey = "82NIHNC5KUPPVLYD";         // channel's Read API Key
unsigned long myTalkBackID = 37638;
const String myTalkBackKey = "5E6QWLXO139URD9M";
const int updateThingSpeakInterval = 5 * 1000; // 20 second interval at which to update ThingSpeak

// Variable Setup
long lastConnectionTime = 0;
boolean lastConnected = false;

// Initialize Arduino Ethernet Client
WiFiClient client;

long int read_data_update_delay = 1000;
unsigned long read_time;

//----------------------SPI---------------------------------------------------------
void SERCOM1_Handler();


volatile uint8_t dataCount = 52;
uint8_t readData = 0;
//-----------------------------------------------------------------------------------

void setup() {
	//Initialize serial and wait for port to open:
	Serial.begin(115200);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}
	// check for the WiFi module:
	if (WiFi.status() == WL_NO_MODULE) {
		Serial.println("Communication with WiFi module failed!");
		// don't continue
		while (true);
	}

	String fv = WiFi.firmwareVersion();
	if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
		Serial.println("Please upgrade the firmware");
	}

	// attempt to connect to Wifi network:
	while (status != WL_CONNECTED) {
		Serial.print("Attempting to connect to open SSID: ");
		Serial.println(ssid);
		status = WiFi.begin(ssid);

		// wait 10 seconds for connection:
		delay(10000);
	}

	// you're connected now, so print out the data:
	Serial.print("You're connected to the network");
	printWifiStatus();
	pinMode(SS, INPUT);
	pinMode(MISO, OUTPUT);
	pinMode(MOSI, INPUT);
	pinMode(SCK, INPUT);
	pinMode(A5, OUTPUT);
	digitalWrite(A5, 0);
	pinMode(A6, OUTPUT);
	digitalWrite(A6, 1);
	pinMode(A7, OUTPUT);
	digitalWrite(A7, 0);
	pinMode(A0, OUTPUT);
	digitalWrite(A0, 1);

	pinMode(slaveAPin, INPUT_PULLUP);
	attachInterrupt(slaveAPin, SERCOM1_Handler, FALLING);
}

void loop() {
	// read values from pins and store as strings


	String light = String(analogRead(A0), DEC); // read light value

	// find temp value
	float voltage = analogRead(A1) * (3.3 / 1024);  // convert 0-1023 range to 3.3V range
	float tempVal = (voltage - 0.5) * 100;            // convert voltage to temperature in *C
	String temp = String(tempVal);

	// Print Update Response to Serial Monitor
	if (client.available()) {
		//char c = client.read();
		//Serial.print(c);
	}
	// Disconnect from ThingSpeak
	if (!client.connected() && lastConnected) {
		client.stop();
	}
	// Update ThingSpeak
	if (!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval)) {
		updateThingSpeak("field1=" + light + "&field2=" + temp + "&field3=" + light + " &field4=" + temp + "&field5=" + light + " &field6=" + temp + "&field7=" + light + " &field8=" + temp);
	}

	lastConnected = client.connected();

	if ((millis() - read_time) > read_data_update_delay) {
		// Create the TalkBack URI
		String tbURI = String("/talkbacks/") + String(myTalkBackID) + String("/commands/execute");
		String postMessage = String("api_key=") + String(myTalkBackKey); // Create the message body for the POST out of the values
		String newCommand = String(); // Make a string for any commands in the queue
		int x = httpPOST(tbURI, postMessage, newCommand); // Make the POST to ThingSpeak
		read_time = millis();
		if (x == 200) { // Check the result
			// check for a command returned from TalkBack
			if (newCommand.length() != 0) {
				if (newCommand == "x_s") { // _s = start, -e = end

				}
				else if (newCommand == "y_s") {
				}
				else if (newCommand == "z_s") {
				}
				else if (newCommand == "w_s") {
				}
				else if (newCommand == "j1_s") {
				}
				else if (newCommand == "j2_s") {
				}
				else if (newCommand == "j3_s") {
				}
				else if (newCommand == "j4_s") {
				}
				else if (newCommand == "x-_s") {
				}
				else if (newCommand == "y-_s") {
				}
				else if (newCommand == "z-_s") {
				}
				else if (newCommand == "w-_s") {
				}
				else if (newCommand == "j1-_s") {
				}
				else if (newCommand == "j2-_s") {
				}
				else if (newCommand == "j3-_s") {
				}
				else if (newCommand == "j4-_s") {
				}
				else if (newCommand == "x_e") {
				}
				else if (newCommand == "y_e") {
				}
				else if (newCommand == "z_e") {
				}
				else if (newCommand == "w_e") {
				}
				else if (newCommand == "j1_e") {
				}
				else if (newCommand == "j2_e") {
				}
				else if (newCommand == "j3_e") {
				}
				else if (newCommand == "j4_e") {
				}
				else if (newCommand == "x-_e") {
				}
				else if (newCommand == "y-_e") {
				}
				else if (newCommand == "z-_e") {
				}
				else if (newCommand == "w-_e") {
				}
				else if (newCommand == "j1-_e") {
				}
				else if (newCommand == "j2-_e") {
				}
				else if (newCommand == "j3-_e") {
				}
				else if (newCommand == "j4-_e") {
				}
				else {
					//Trap
				}
			}
		}
		else {
			Serial.println("Problem checking queue. HTTP error code " + String(x));
		}
	}
	
}

void updateThingSpeak(String tsData) {
	if (client.connect(thingSpeakAddress, 80)) {
		client.print("POST /update HTTP/1.1\n");
		client.print("Host: api.thingspeak.com\n");
		client.print("Connection: close\n");
		client.print("X-THINGSPEAKAPIKEY: " + APIKey + "\n");
		client.print("Content-Type: application/x-www-form-urlencoded\n");
		client.print("Content-Length: ");
		client.print(tsData.length());
		client.print("\n\n");
		client.print(tsData);
		lastConnectionTime = millis();
	}
}

void printWifiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
}

int httpPOST(String uri, String postMessage, String& response) {

	bool connectSuccess = false;
	connectSuccess = client.connect(thingSpeakAddress, 80);

	if (!connectSuccess) {
		return -301;
	}

	postMessage += "&headers=false";

	String Headers = String("POST ") + uri + String(" HTTP/1.1\r\n") +
		String("Host: api.thingspeak.com\r\n") +
		String("Content-Type: application/x-www-form-urlencoded\r\n") +
		String("Connection: close\r\n") +
		String("Content-Length: ") + String(postMessage.length()) +
		String("\r\n\r\n");

	client.print(Headers);
	client.print(postMessage);

	long startWaitForResponseAt = millis();
	while (client.available() == 0 && millis() - startWaitForResponseAt < 5000) {
		delay(100);
	}

	if (client.available() == 0) {
		return -304; // Didn't get server response in time
	}

	if (!client.find(const_cast<char*>("HTTP/1.1"))) {
		return -303; // Couldn't parse response (didn't find HTTP/1.1)
	}

	int status = client.parseInt();
	if (status != 200) {
		return status;
	}

	if (!client.find(const_cast<char*>("\n\r\n"))) {
		return -303;
	}

	String tempString = String(client.readString());
	response = tempString;

	return status;

}


void SERCOM1_Handler() {
	data = (uint8_t)SERCOM1->SPI.DATA.reg;
	uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; //Read SPI interrupt register
	if (interrupts & (1 << 3)) {
		Serial.println("SPI SSL Interupt");
		SERCOM1->SPI.INTFLAG.bit.SSL = 1; //clear slave select interrupt
	}
	if (interrupts & (1 << 2)) {
		Serial.println("SPI Data Received Complete Interrupt");
		data = SERCOM1->SPI.DATA.reg; //Read data register
		Serial.print("DATA: "); Serial.println(data);
		SERCOM1->SPI.INTFLAG.bit.RXC = 1; //clear receive complete interrupt
		SERCOM1->SPI.INTFLAG.bit.TXC = 1; //clear Transmit complete interrupt
	}

	if (interrupts & (1 << 1)) {
		Serial.println("SPI Data Transmit Complete Interrupt");
		SERCOM1->SPI.INTFLAG.bit.TXC = 1; //clear Transmit complete interrupt
	}

	if (interrupts & (1 << 0)) {
		Serial.println("SPI Data Register Empty Interrupt");
		SERCOM1->SPI.DATA.reg = 0xAA;
	}
	Serial.print("DATA: "); Serial.println((char)data);
	interrupts();
}