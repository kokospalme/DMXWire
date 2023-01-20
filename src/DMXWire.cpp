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
nrf24Data_t DMXWire::nrf24;
RF24 *DMXWire::radio;

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
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
	Wire.begin(sda, scl,clock);
	DMXWire::slaveAddress = slaveaddress;
}

void DMXWire::beginSlaveRX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   Serial.println("begin Slave RX");
   // sync_dmx = xSemaphoreCreateMutex(); //create semaphore
	Wire.begin(slaveaddress, sda, scl,clock);
	Wire.onReceive(DMXWire::slaveRXcallback); // register event
	DMXWire::slaveAddress = slaveaddress;
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

   // xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
	return packets[_packetNo][DMXWIRE_HEAD + _byteNo];
   // xSemaphoreGive(sync_dmx);
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


   // Serial.println("take Semaphore...");
   // delay(50);
   xSemaphoreTake(sync_dmx,portMAX_DELAY);   //task safety
	packets[_packetNo][DMXWIRE_HEAD + _byteNo] = value;
   xSemaphoreGive(sync_dmx);
}

void DMXWire::masterTXcallback(){
	if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, HIGH);

	for(int i = 0; i < DMXWIRE_PACKETS; i++){	//ToDo: später mehr
		packetNo = i;

      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		packets[i][0] = packetNo;	//head: info which packet is being send
      xSemaphoreGive(sync_dmx);

		sendPacket();	//send packet
      
	}

	if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::slaveRXcallback(int bufSize){  //callback for Wire slave

	uint8_t _counter = 0;
	uint8_t buffer[bufSize];
	
	for(int i=0; i < bufSize; i++){
      // xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		buffer[i] = Wire.read();
      

		if(buffer[0] == 0){
			timestamp_wire = millis();
			if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, HIGH);
		}
      // xSemaphoreGive(sync_dmx);
		// Serial.print(buffer[i], HEX);
	}
	// Serial.println("");


	if(bufSize > DMXWIRE_BYTES_PER_PACKET) return;	//return if buffer too large
	packetNo = buffer[0];
	

	if(packetNo < DMXWIRE_PACKETS){	// if packet in packetrange: write buffer to packet

		if(buffer[0] == DMXWIRE_PACKETS-1){
			duration = millis() - timestamp_wire;
			if(config.ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledRxpin, LOW);
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
	if(packetNo == 0)Serial.printf("P%u: %u \t%u \t%u \t%u \t%u\n", packets[packetNo][0], packets[packetNo][1], packets[packetNo][2], packets[packetNo][3], packets[packetNo][4], packets[packetNo][5]);
}


DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
}



void DMXWire::dmxboardInit(){
   config.ledRxpin = LEDRX_PIN;
   config.ledTxpin = LEDTX_PIN;
   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);

   	switch(config.ioMode){	//input/output mode
         case DMXBOARD_MODE_OFF:	//mode off [Wire slave, no output]
         beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_OFF);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("DMX board OFF");
         break;

         case DMXBOARD_MODE_TX_DMX512:	//mode tx dmx512 [Wire slave, DMX TX]
         beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_DMX512);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("DMX board TX DMX512 Mode");
         DMX::Initialize(output);
         break;

         case DMXBOARD_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
         beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("DMX board TX NRF24 Mode");
         nrf24InitTX();   //init NRF24
         break;

         case DMXBOARD_MODE_RX_DMX512:	//mode rx dmx512 [Wire slave, DMX RX]
         // beginSlaveTX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("DMX board RX DMX512 Mode");
         DMX::Initialize(input);
         break;

         case DMXBOARD_MODE_RX_NRF24:	//mode rx dmx512 [Wire slave, DMX RX]
         // beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_WIRE);   //ToDo: implement
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("DMX board RX NRF24 Mode");
         break;

         case DMXBOARD_MODE_DMX512TONRF24:   //ToDo: implement
         // beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("DMX board DMX512 to NRF24 Mode");
         break;

         case DMXBOARD_MODE_NRF24TODMX512:   //ToDo: implement
         setLedTx(DMXWIRE_LED_DMX512);
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("DMX board NRF24 to DMX512 Mode");
         break;

         default:	//default [Wire slave, no output]
         setLedTx(DMXWIRE_LED_OFF);
         setLedRx(DMXWIRE_LED_WIRE);
         break;
	}
}

void DMXWire::dmxboardRun(){
   switch(config.ioMode){
      case DMXBOARD_MODE_OFF:
      break;
      case DMXBOARD_MODE_TX_DMX512:
         for(int i = 1; i <= 512; i++){   //write all 521 bytes from Wire to DMX Serial output
            DMX::Write(i, read(i-5));
         }
      break;
      case DMXBOARD_MODE_TX_NRF24:
         nrf24TX();  //Wire-packets to NRF24
      break;
      default:
      break;
   }
   getTimeout_wire();
   getTimeout_dmx512();
   getTimeout_wire();
}




void DMXWire::nrf24InitTX(){
   radio->begin();
   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(nrf24.RXTXchannel); // set the channel
   radio->openWritingPipe(nrf24.RXTXaddress); // set network address
   radio->stopListening(); // start talking !
}

void DMXWire::nrf24InitRX(){
   radio->begin();
   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(nrf24.RXTXchannel); // set the channel
   radio->openReadingPipe(1,nrf24.RXTXaddress); // set network address
   radio->startListening(); // start listening for data
}

void DMXWire::nrf24TX() { //transmit via NRF24

   for (uint8_t group = 0; group < NRF24_MAXGROUPS; group++) { // send groups of DMX data, 16 bytes at a time
      uint16_t group_ptr = group * NRF24_BYTES_PER_PACKET; // create group pointer for array
      if (millis() > timestamp_nrf24+ config.timeout_nrf24_ms) { // allow ALL radio data (full DMX array) to be send once per second, regardless of changes
         timestamp_nrf24 = millis(); // reset timestamp
         nrf24.group_send = true; // force ALL send
      }else { 
         nrf24.group_send = false; // preset flag to false, only set it if there has been a data change since last time
      } 


      for (uint8_t chan = 0; chan < NRF24_BYTES_PER_PACKET; chan++) {
         if ( Dmxwire.read(group_ptr+chan) != nrf24.shadow_DMX[group_ptr+chan] ) { // compare test : current DMX against old DMX 
            nrf24.shadow_DMX[group_ptr+chan] = Dmxwire.read(group_ptr+chan); // if changed, update shadow array of DMX data and payload
            nrf24.group_send = true; // set flag so that THIS group packet gets sent
         } 
         nrf24.payload[chan+2] = Dmxwire.read(group_ptr+chan); // ensure ALL up-to-date data gets through on this packet
      } 
      

      if (nrf24.group_send) { // only send the data that has changed, any data change in a group will result in whole group send
         nrf24.payload[0] = group; // set first byte to point to group number (groups of 16 bytes)
         nrf24.payload[1] = nrf24.timeStamp++; // second byte helps us monitor consistency of reception at receiver with a continuous frame counter
         radio->write( nrf24.payload, sizeof(nrf24.payload) ); // dump payload to radio
      } 
   } 
} 

void DMXWire::nrf24RX(){
   if ( radio->available() ) {
      if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, HIGH);

      radio->read( nrf24.payload, sizeof(nrf24.payload) ); // get data packet from radio 
      if(config.ioMode == DMXBOARD_MODE_RX_NRF24){
         for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
            uint16_t _channel = NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ;
            // uint8_t _value = nrf24.payload[i+2];
            write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]); // parse radio data into dmx data array
            
         } 
      }
      if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
      
   } 
}