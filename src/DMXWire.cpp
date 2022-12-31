/*
 * DMXWire.cpp
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#include "DMXWire.h"

DMXWire::DMXWire(uint8_t timerNo) {
//	ITtimer = new ESP32Timer(timerNo);	//set timer

}

void DMXWire::setClock(uint32_t frequency){
	Wire.setClock(frequency);
}

void DMXWire::beginSender(uint8_t scl,uint8_t sda, uint8_t slaveaddress){
	Wire.begin(scl, sda);
	slaveAddress = slaveaddress;

//	// Interval in microsecs
//	if (ITtimer.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, IRhandlerTX))
//	{
//		Serial.print(F("Starting  ITimer0 OK, millis() = "));
//		Serial.println(millis());
//	}
//	else
//		Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

	uint16_t _packet = channel / DMXW_PACKETSIZE;
	uint16_t _ch = channel - (_packet * DMXW_PACKETSIZE) -1 ;

	return buffer[_packet][_ch];
}


void DMXWire::write(uint16_t channel, uint8_t byte){	//write byte to dmx Buffer
	uint16_t _packet = channel / DMXW_PACKETSIZE;
	uint16_t _ch = channel - (_packet * DMXW_PACKETSIZE) -1 ;

	buffer[_packet][_ch] = byte;
}
void DMXWire::runSender(){
	if(millis() > timer + TIMER_INTERVAL_MS){
		timer = millis();
		for(int i = 0; i < 16; i++){
			packetNo = i;
			setPacket();
			sendPacket();
		}
	}
}

void DMXWire::runReceiver(){
	if(millis() > timer + TIMER_INTERVAL_MS){
		timer = millis();
		for(int i = 0; i < 16; i++){
			packetNo = i;
			setPacket();
			requestData();
		}
	}
}

//bool DMXWire::IRhandlerTX(void * timerNo){
//	for(int i = 0; i < 16; i++){
//		packetNo = i;
//		setPacket();
//		requestData();
//	}
//}

void DMXWire::requestData(){
  Wire.requestFrom(8, 16);    // request 6 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    uint8_t c = Wire.read(); // receive a byte as character
    Serial.print(c);Serial.print("\t");         // print the character
  }
  Serial.println();
}

void DMXWire::setPacket(){
  Wire.beginTransmission(slaveAddress); // transmit to device #8
  Wire.write(packetNo);              // sends one byte
  Wire.endTransmission();    // stop transmitting
}

void DMXWire::sendPacket(){
  Wire.beginTransmission(slaveAddress); // transmit to device #8
  Wire.write(packetNo);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  Wire.beginTransmission(slaveAddress); // transmit to slave device
  Wire.write(buffer[packetNo], DMXW_PACKETSIZE);              // send buffer
  Wire.endTransmission();    // stop transmitting

}












DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}

