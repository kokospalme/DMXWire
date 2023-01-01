/*
 * DMXWire.cpp
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#include "DMXWire.h"

uint8_t DMXWire::packets[DMXWIRE_PACKETS][DMXWIRE_BYTES_PER_PACKET];
uint8_t DMXWire::packetNo = 0;
uint8_t DMXWire::slaveAddress = 1;	//slave's address
int DMXWire::packetBusy =  DMXWIRE_NOTBUSY;

DMXWire::DMXWire() {
	for(int i = 0; i < DMXWIRE_BYTES_PER_PACKET; i++){
		for(int j = 0; j < DMXWIRE_PACKETS; j++){
			packets[j][i] = 0;
		}
	}
}

void DMXWire::setClock(uint32_t frequency){
	Wire.setClock(frequency);
}

void IRhandler(DMXWire* instance){

}

void DMXWire::beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress){
	Wire.begin(scl, sda);
	DMXWire::slaveAddress = slaveaddress;
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

return 0;
}

void DMXWire::write(uint16_t channel, uint8_t value){
	if(channel < 1 || channel > 512) return;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

	packets[_packetNo][DMXWIRE_HEAD + _byteNo] = value;
}

void DMXWire::masterTXcallback(){
	for(int i = 0; i < 1; i++){	//ToDo: später mehr
		packetNo = i;
		packets[i][0] = packetNo;
		// setPacket();	//send slave, which packet is being send
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
	if(packetNo == 0)Serial.printf("%u \t%u \t%u \t%u \t%u \n", packets[packetNo][0], packets[packetNo][1], packets[packetNo][2], packets[packetNo][3], packets[packetNo][4]);
}


DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}

