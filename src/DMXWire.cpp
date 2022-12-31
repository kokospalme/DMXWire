/*
 * DMXWire.cpp
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#include "DMXWire.h"

uint8_t DMXWire::packets[DMXWIRE_BYTES_PER_PACKET][DMXWIRE_PACKETS];
uint8_t DMXWire::packetNo = 0;
uint8_t DMXWire::slaveAddress = 1;	//slave's address
int DMXWire::packetBusy =  DMXWIRE_NOTBUSY;

DMXWire::DMXWire() {

}

void DMXWire::setClock(uint32_t frequency){
	Wire.setClock(frequency);
}
void IRhandler(DMXWire* instance){

}

void DMXWire::beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress){
	Wire.begin(scl, sda);
	DMXWire::slaveAddress = slaveaddress;

	// // esp_timer_create(&_timerConfig, &IRtimer);
	// IRtimer.attach(DMXWIRE_INTERVAL_MS, IRhandler);

	// if (IRtimer.active())
	// {
	// 	Serial.print(F("Starting timer IRsend_DMX OK, millis() = "));
	// 	Serial.println(millis());
	// }
	// else
	// 	Serial.println(F("Can't set timer to IRsend_DMX"));
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

return 0;
}

void DMXWire::write(uint16_t channel, uint8_t byte){

}

void DMXWire::masterTXcallback(){
	
	for(int i = 0; i < DMXWIRE_PACKETS; i++){
		packetNo = i;
		setPacket();	//send slave, which packet is being send
		sendPacket();	//send packet
	}
}

void DMXWire::setPacket(){	//master TX
	Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(packetNo);              // sends one byte
	Wire.endTransmission();    // stop transmitting
}

void DMXWire::sendPacket(){	//master TX
	packetBusy = packetNo;
	Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(packets[packetNo], DMXWIRE_BYTES_PER_PACKET);
	Wire.endTransmission();    // stop transmitting
	packetBusy = DMXWIRE_NOTBUSY;
}


DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}

