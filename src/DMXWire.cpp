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
unsigned long DMXWire::duration = 0;
unsigned long DMXWire::timestamp_wire = 0;
unsigned long DMXWire::timestamp_dmx512 = 0;
unsigned long DMXWire::timestamp_nrf24 = 0;
dmxwire_settings_t DMXWire::config;
SemaphoreHandle_t DMXWire::sync_dmx;
RF24 DMXWire::radio;

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

void DMXWire::setLedRx(int pin, uint8_t mode){	//set pin and mode for ledRx
	config.ledRxpin = pin;
	config.ledRxMode = mode;
	pinMode(config.ledRxpin, OUTPUT);
	digitalWrite(config.ledRxpin, LOW);
}

void DMXWire::setLedRx(uint8_t mode){	//set mode for ledRx
	config.ledRxMode = mode;
	pinMode(config.ledRxpin, OUTPUT);
	digitalWrite(config.ledRxpin, LOW);
}

void DMXWire::setLedTx(int pin, uint8_t mode){	//set pin and mode for ledTx
	config.ledTxpin = pin;
	config.ledTxMode = mode;
	pinMode(config.ledTxpin, OUTPUT);
	digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::setLedTx(uint8_t mode){	//set mode for ledRx
	config.ledTxMode = mode;
	pinMode(config.ledTxpin, OUTPUT);
	digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
	Wire.begin(sda, scl,clock);
	DMXWire::slaveAddress = slaveaddress;
}

void DMXWire::beginSlaveRX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
	Wire.begin(slaveaddress, sda, scl,clock);
	Wire.onReceive(DMXWire::slaveRXcallback); // register event
	DMXWire::slaveAddress = slaveaddress;
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

   xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
	return packets[_packetNo][DMXWIRE_HEAD + _byteNo];
   xSemaphoreGive(sync_dmx);
}

unsigned long DMXWire::getDuration(){
	return duration;
}

void DMXWire::setTimout_wire(unsigned long timeout_ms){
	config.timeout_wire_ms = timeout_ms;
}

void DMXWire::setTimout_dmx512(unsigned long timeout_ms){
	config.timeout_dmx512_ms = timeout_ms;
}

void DMXWire::setTimout_nrf24(unsigned long timeout_ms){
	config.timeout_nrf24_ms = timeout_ms;
}

bool DMXWire::getTimeout_wire(){
	if(millis() > timestamp_wire + config.timeout_wire_ms){
		if(config.ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

bool DMXWire::getTimeout_dmx512(){
	if(millis() > timestamp_dmx512 + config.timeout_dmx512_ms){
		if(config.ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

bool DMXWire::getTimeout_nrf24(){
	if(millis() > timestamp_nrf24 + config.timeout_nrf24_ms){
		if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

void DMXWire::write(uint16_t channel, uint8_t value){
	if(channel < 1 || channel > 512) return;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

	packets[_packetNo][DMXWIRE_HEAD + _byteNo] = value;
}

void DMXWire::masterTXcallback(){
	if(config.ledRxMode == DMXWIRE_LED_TX) digitalWrite(config.ledRxpin, HIGH);
	if(config.ledTxMode == DMXWIRE_LED_TX) digitalWrite(config.ledTxpin, HIGH);

	for(int i = 0; i < DMXWIRE_PACKETS; i++){	//ToDo: später mehr
      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		packetNo = i;
		packets[i][0] = packetNo;	//head: info which packet is being send
		sendPacket();	//send packet
      xSemaphoreGive(sync_dmx);
	}

	if(config.ledRxMode == DMXWIRE_LED_TX) digitalWrite(config.ledRxpin, LOW);
	if(config.ledTxMode == DMXWIRE_LED_TX) digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::slaveRXcallback(int bufSize){


	uint8_t _counter = 0;
	uint8_t buffer[bufSize];
	
	for(int i=0; i < bufSize; i++){
      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		buffer[i] = Wire.read();
      

		if(buffer[0] == 0){
			timestamp = millis();
			if(config.ledRxMode == DMXWIRE_LED_RX) digitalWrite(config.ledRxpin, HIGH);
			if(config.ledTxMode == DMXWIRE_LED_RX) digitalWrite(config.ledTxpin, HIGH);
		}
      xSemaphoreGive(sync_dmx);
		// Serial.print(buffer[i], HEX);
	}
	// Serial.println("");


	if(bufSize > DMXWIRE_BYTES_PER_PACKET) return;	//return if buffer too large
	packetNo = buffer[0];
	

	if(packetNo < DMXWIRE_PACKETS){	// if packet in packetrange: write buffer to packet

		if(buffer[0] == DMXWIRE_PACKETS-1){
			duration = millis() - timestamp;
			if(config.ledRxMode == DMXWIRE_LED_RX) digitalWrite(config.ledRxpin, LOW);
			if(config.ledTxMode == DMXWIRE_LED_RX) digitalWrite(config.ledTxpin, LOW);
		}

			for(int i = 0; i < DMXWIRE_BYTES_PER_PACKET; i++){
				packets[packetNo][i] = buffer[i];
			}

			// if(packetNo== 0){
			// 	Serial.print(Dmxwire.getDuration());
			// 	Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", packets[0][1], packets[0][2], packets[0][3], packets[0][4], packets[0][5]);
			// }

			
	}else{	//else: setting codes or unknown
		switch(packetNo){
			case 250:	//ToDo: codes für settings etc
			Serial.println("Setting XY changed...");
			break;
			default:
			Serial.println("unknown command");
			break;
		}
	}

	packetNo = DMXWIRE_NOTBUSY;	//reset "busy-variable"

}

void DMXWire::setPacket(){	//master TX
	Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(packetNo);              // sends one byte
	Wire.endTransmission();    // stop transmitting
}

void DMXWire::sendPacket(){	//master TX
   xSemaphoreTake(sync_dmx, portMAX_DELAY);  
	packetBusy = packetNo;
   uint8_t _packet[DMXWIRE_BYTES_PER_PACKET];
   for(int i = 0; i <DMXWIRE_BYTES_PER_PACKET; i++){
      _packet[i] = packets[packetNo][i];
   }
   xSemaphoreGive(sync_dmx);

	Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(_packet, DMXWIRE_BYTES_PER_PACKET);
	Wire.endTransmission();    // stop transmitting
	packetBusy = DMXWIRE_NOTBUSY;
	if(packetNo == 0)Serial.printf("%u \t%u \t%u \t%u \t%u \n", packets[packetNo][0], packets[packetNo][1], packets[packetNo][2], packets[packetNo][3], packets[packetNo][4]);
}


DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}

