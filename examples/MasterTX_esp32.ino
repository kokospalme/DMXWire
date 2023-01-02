/*	LC1 DMXWire example
 *
 *	Example: Master TX
 *
 */
#include "Arduino.h"
#include <DMXWire.h>
//#include <LC1.h>
//#include <FastLED.h>
//#include <WiFi.h>
//#include <tinyXML.h>

#define I2C_CLOCK 400000	// ToDo: 100000??
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 16
#define SDA_PIN 22
#define DMXWIRE_TIMER 0	//ESP32: 0... ?

#define ANALOG0_PIN 33
#define ANALOG1_PIN 32	//ToDo: später 32 (wo jetzt A5 ist)
#define ANALOG2_PIN 39	//SENSOR_VN
#define ANALOG3_PIN 34
#define ANALOG4_PIN 35

Ticker dmxTicker;

void dmxCallback(){
	DMXWire* instance;
	instance->masterTXcallback();
}

void setup(){
	
	Serial.begin(115200);
	Serial.println("*** DMXWire Master TX starting ***");
	dmxTicker.attach_ms(DMXWIRE_INTERVAL_MS, dmxCallback);
	if (dmxTicker.active())
	{
		Serial.print(F("Starting timer IRsend_DMX OK, millis() = "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set timer to IRsend_DMX"));

	
	Dmxwire.beginMasterTX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK); // ToDo: github mögliche Lösung: https://github.com/espressif/arduino-esp32/issues/3465
}

void loop(){

	Dmxwire.write(1, map(analogRead(ANALOG0_PIN),0,4095,0,255));
	Dmxwire.write(2, map(analogRead(ANALOG1_PIN),0,4095,0,255));
	Dmxwire.write(3, map(analogRead(ANALOG2_PIN),0,4095,0,255));
	Dmxwire.write(4, map(analogRead(ANALOG3_PIN),0,4095,0,255));
	Dmxwire.write(5, map(analogRead(ANALOG4_PIN),0,4095,0,255));
	// Serial.printf("%u \t%u \t%u \t%u \t%u \n", analogRead(ANALOG0_PIN), analogRead(ANALOG1_PIN), analogRead(ANALOG2_PIN), analogRead(ANALOG3_PIN), analogRead(ANALOG4_PIN));
	// 	delay(50);
}



