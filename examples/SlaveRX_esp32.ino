/*	LC1 DMXWire example
 *
 *	Example: Slave RX
 *
 * Testing timings:
 *  - 16...18ms für alle packets bei Clock=400000 (1 Frame mit 512 Bytes + 1 byte head)
 */
#include "Arduino.h"
#include <DMXWire.h>

#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 5
#define SDA_PIN 4

#define LED0_PIN 12	//Luatosboard: 12 & 13
#define LED1_PIN 13

void setup(){
	Serial.begin(115200);
	Serial.println("*** DMXWire Slave RX starting ***");

	// Wire.begin(DMXWIRE_SLAVEADDRESS, SDA_PIN, SCL_PIN, I2C_CLOCK);
	// Wire.onReceive(slaveRXcallback);
	Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
	Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
}

void loop(){

	Serial.print(Dmxwire.getDuration());
	Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", Dmxwire.read(1), Dmxwire.read(2), Dmxwire.read(3), Dmxwire.read(4), Dmxwire.read(5));
	delay(50);

	Dmxwire.getTimeout();	//check if slave gets data 
}



