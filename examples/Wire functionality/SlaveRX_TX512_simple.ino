/*	LC1 DMXWire example
 *
 *	Example: Slave RX
 *
 * Testing timings:
 *  - 16...18ms für alle packets bei Clock=400000 (1 Frame mit 512 Bytes + 1 byte head)
 */
#include "Arduino.h"
#include <DMXWire.h>
#include <dmx.h>

#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 5
#define SDA_PIN 4


uint8_t send_value = 0;

void setup(){
	Serial.begin(115200);
	Serial.println("*** DMXWire Slave RX starting ***");

	DMX::Initialize(output);
	// Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
	Dmxwire.beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
   Serial.println("setup finished");
}

void loop(){
	
	for(int i = 1; i <= 512; i++){
		DMX::Write(i, Dmxwire.read(i-5));
	}
	
	Serial.print(Dmxwire.getDuration());
	Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", Dmxwire.read(1), Dmxwire.read(2), Dmxwire.read(3), Dmxwire.read(4), Dmxwire.read(5));
	delay(30);

	Dmxwire.getTimeout_wire();	//check if slave gets data 
}
