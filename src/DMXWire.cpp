/*
 * DMXWire.cpp
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#include "DMXWire.h"

DMXWire::DMXWire(uint8_t timerNo) {
	ITtimer = new ESP32Timer(timerNo);	//set timer

}

void DMXWire::setClock(uint32_t frequency){
	Wire.setClock(frequency);
}

void DMXWire::beginSender(uint8_t scl,uint8_t sda, uint8_t slaveaddress){
	Wire.begin(scl, sda);
	slaveAddress = slaveaddress;

	// Interval in microsecs
	if (ITtimer.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, IRhandlerTX))
	{
		Serial.print(F("Starting  ITimer0 OK, millis() = "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

}

bool DMXWire::IRhandlerTX(void * timerNo){
	for(int i = 0; i < 16; i++){
		packetNo = i;
		setPacket();
		requestData();
	}
}
















DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}

