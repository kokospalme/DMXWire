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
SemaphoreHandle_t DMXWire::sync_config;
nrf24Data_t DMXWire::nrf24;
Preferences *DMXWire::preferences;
RF24 *DMXWire::radio;

DMXWire::DMXWire() {

	for(int i = 0; i < DMXWIRE_BYTES_PER_PACKET; i++){
		for(int j = 0; j < DMXWIRE_PACKETS; j++){
			packets[j][i] = 0;
		}
	}
}

DMXWire::~DMXWire() {
	// TODO Auto-generated destructor stub
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

void DMXWire::setIomode(uint8_t mode){
   if(mode != config.ioMode){
      config.ioMode = mode;
   }
}

void DMXWire::beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
	Wire.begin(sda, scl,clock);
	DMXWire::slaveAddress = slaveaddress;
}

void DMXWire::beginSlaveRX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   Serial.println("begin Slave RX");
	Wire.begin(slaveaddress, sda, scl,clock);
	Wire.onReceive(DMXWire::slaveRXcallback); // register event
	DMXWire::slaveAddress = slaveaddress;
}

uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

   xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
   uint8_t _value = packets[_packetNo][DMXWIRE_HEAD + _byteNo];
   xSemaphoreGive(sync_dmx);

	return _value;
   
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

		Dmxwire.sendPacket();	//send packet
      
	}

	if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::slaveRXcallback(int bufSize){  //callback for Wire slave

	uint8_t _counter = 0;
	uint8_t buffer[bufSize];
	
	for(int i=0; i < bufSize; i++){
      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		buffer[i] = Wire.read();
      

		if(buffer[0] == 0){
			timestamp_wire = millis();
		}
      if(config.ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledRxpin, HIGH);
      xSemaphoreGive(sync_dmx);
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
	// if(packetNo == 0)Serial.printf("P%u: %u \t%u \t%u \t%u \t%u\n", packets[packetNo][0], packets[packetNo][1], packets[packetNo][2], packets[packetNo][3], packets[packetNo][4], packets[packetNo][5]);
}



void DMXWire::dmxboardInit(){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   Serial.println("read config from preferences:");

   Dmxwire.preferencesInit();
   Dmxwire.readConfig();

   config.ledRxpin = LEDRX_PIN;  //overwrite pins
   config.ledTxpin = LEDTX_PIN;
   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);

   	switch(config.ioMode){	//input/output mode
         case DMXBOARD_MODE_OFF:	//mode off [Wire slave, no output]
            Serial.println("init MODE_OFF");
            Dmxwire.beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            Dmxwire.setLedTx(DMXWIRE_LED_OFF);
            Dmxwire.setLedRx(DMXWIRE_LED_WIRE);
            
         break;

         case DMXBOARD_MODE_TX_DMX512:	//mode tx dmx512 [Wire slave, DMX TX]
            Serial.println("init MODE_TX_DMX512");
            DMX::Initialize(output);
            Dmxwire.beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            Dmxwire.setLedTx(DMXWIRE_LED_DMX512);
            Dmxwire.setLedRx(DMXWIRE_LED_WIRE);
            
         break;

         case DMXBOARD_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
            Serial.println("init MODE_TX_NRF24");
            DMX::Initialize(input);
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_NRF24);
            setLedRx(DMXWIRE_LED_WIRE);
            nrf24InitTX();   //init NRF24
            // xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24_tx_task", 2048, NULL, 1, NULL, NRF24_CORE);
         break;

         case DMXBOARD_MODE_RX_DMX512:	//mode rx dmx512 [Wire slave, DMX RX]
            // beginSlaveTX(SCL_PIN, SDA_PIN, config.slaveAddress, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_WIRE);
            setLedRx(DMXWIRE_LED_DMX512);
            Serial.println("DMX board RX DMX512 Mode");
            DMX::Initialize(input);
         break;

         case DMXBOARD_MODE_RX_NRF24:	//mode rx dmx512 [Wire slave, DMX RX]
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
            setLedTx(DMXWIRE_LED_WIRE);   //ToDo: implement
            setLedRx(DMXWIRE_LED_NRF24);
            Serial.println("DMX board RX NRF24 Mode");
         break;

         case DMXBOARD_MODE_DMX512TONRF24:   //ToDo: implement
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_NRF24);
            setLedRx(DMXWIRE_LED_DMX512);
            DMX::Initialize(input);
            Serial.println("DMX board DMX512 to NRF24 Mode");
         break;

         case DMXBOARD_MODE_NRF24TODMX512:
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_DMX512);
            setLedRx(DMXWIRE_LED_NRF24);
            DMX::Initialize(output);
            xTaskCreatePinnedToCore(DMXWire::nrf24RX_toDmx512, "nrf24_rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
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
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, HIGH);
         for(int i = 1; i <= 512; i++){   //write all 521 bytes from Wire to DMX Serial output

            DMX::Write(i, Dmxwire.read(i));
         }
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
         Serial.printf("MODE_TX_DMX512. ch1:%u\n",DMX::Read(1));  //read from Dmx buffer
         delay(20);

      break;
      case DMXBOARD_MODE_TX_NRF24:
         // Dmxwire.nrf24TX();  //Wire-packets to NRF24 obsolet
         while(nrf24.radioOK == false){}  //wait for nrf24 module to initialize
         Serial.printf("MODE_TX_NRF24. ch1:%u\n",nrf24.shadow_DMX[1]);  //read from nrf24 buffer
         delay(15);
      break;
      case DMXBOARD_MODE_RX_DMX512:
         // Dmxwire.nrf24RX();
         // Serial.println(Dmxwire.read(1));

         Serial.println("MODE_RX_DMX512");
         delay(500);
      break;

      case DMXBOARD_MODE_RX_NRF24:
         // Dmxwire.nrf24RX();
         // Serial.println(Dmxwire.read(1));

         Serial.printf("MODE_RX_NRF24. ch1:%u\n",Dmxwire.read(1));  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_DMX512TONRF24:


         // for(int i = 1; i <= 512; i++){   //write all 521 bytes from Wire to DMX Serial output
         //    Dmxwire.write(i, DMX::Read(i));  //read from Serial to Buffer
         // }
         // Dmxwire.nrf24TX();
         Serial.println("MODE_DMX512TONRF24");
         delay(500);
      break;
      case DMXBOARD_MODE_NRF24TODMX512:
         // if(config.ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, HIGH);
         // if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, HIGH);

      //    Dmxwire.nrf24RX();
      //    for(int i = 1; i <= 512; i++){   //write all 521 bytes from Wire to DMX Serial output
      //       DMX::Write(i, Dmxwire.read(i));
      //    }
      //    // Serial.println(Dmxwire.read(1));
      //    if(config.ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
      //    if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, LOW);
      //    delay(10);
      
         Serial.println("DMXBOARD_MODE_NRF24TODMX512");
         delay(500);
      break;

      default:
      break;
   }
   getTimeout_wire();
   getTimeout_dmx512();
   getTimeout_wire();
   Dmxwire.serialhandler();
}


void DMXWire::nrf24InitTX(){
   nrf24.radioOK = false;
   Serial.println("NRF24 initializing (TX)...");
   if(!radio->begin()){
      Serial.println("ERROR: nrf24 module is not responding");
      return;
   }
   Serial.println("NRF24 OK.");
   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(config.nrf_RXTXchannel); // set the channel
   radio->openWritingPipe(config.nrf_RXTXaddress); // set network address
   radio->stopListening(); // start talking !
   Serial.println("NRF24 init OK.");
   nrf24.radioOK = true;
}

void DMXWire::nrf24InitRX(){
   nrf24.radioOK = false;
   Serial.println("NRF24 initializing (TX)...");
   if(!radio->begin()){
      Serial.println("ERROR: nrf24 module is not responding");
      return;
   }
   Serial.println("NRF24 OK.");
   

   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(config.nrf_RXTXchannel); // set the channel
   radio->openReadingPipe(1,config.nrf_RXTXaddress); // set network address
   radio->startListening(); // start listening for data
   Serial.println("NRF24 init OK.");
   nrf24.radioOK = true;
}

void DMXWire::nrf24TX(void*pvParameters) { //transmit via NRF24
   if(nrf24.radioOK == false) return;  //return if nrf24 is not initialized

   

   for (uint8_t group = 0; group < NRF24_MAXGROUPS; group++) { // send groups of DMX data, 16 bytes at a time
      uint16_t group_ptr = group * NRF24_BYTES_PER_PACKET; // create group pointer for array
      if (millis() > timestamp_nrf24 + config.timeout_nrf24_ms) { // allow ALL radio data (full DMX array) to be send once per second, regardless of changes
         timestamp_nrf24 = millis(); // reset timestamp
         nrf24.group_send = true; // force ALL send
      }else { 
         nrf24.group_send = false; // preset flag to false, only set it if there has been a data change since last time
      } 


      for (uint8_t chan = 1; chan < NRF24_BYTES_PER_PACKET+1; chan++) {
         if ( Dmxwire.read(group_ptr+chan-1) != nrf24.shadow_DMX[group_ptr+chan-1] ) { // compare test : current DMX against old DMX 
            nrf24.shadow_DMX[group_ptr+chan-1] = Dmxwire.read(group_ptr+chan-1); // if changed, update shadow array of DMX data and payload
            nrf24.group_send = true; // set flag so that THIS group packet gets sent
         } 
         nrf24.payload[chan+1] = Dmxwire.read(group_ptr + chan-1); // ensure ALL up-to-date data gets through on this packet
      } 
      

      if (nrf24.group_send) { // only send the data that has changed, any data change in a group will result in whole group send
      if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, HIGH);
         nrf24.payload[0] = group; // set first byte to point to group number (groups of 16 bytes)
         nrf24.payload[1] = nrf24.timeStamp++; // second byte helps us monitor consistency of reception at receiver with a continuous frame counter
         // Serial.println("radio OK. send payload...");
         radio->write( nrf24.payload, sizeof(nrf24.payload) ); // dump payload to radio
      } 
   } 
   if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, LOW);

   xSemaphoreTake(sync_config, portMAX_DELAY);
   uint8_t _iomode = config.ioMode;
   xSemaphoreGive(sync_config);
   if(_iomode != DMXBOARD_MODE_TX_NRF24){ //if ioMode is not longer TX NRF24, delete task
      vTaskDelete(NULL);
   }
} 

void DMXWire::nrf24rx_task(void*pvParameters){

   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio

   nrf24.radioOK = false;
   Serial.println("NRF24 initializing (TX)...");
   if(!radio->begin()){
      Serial.println("ERROR: nrf24 module is not responding. quit task");
      vTaskDelete(NULL);
   }
   Serial.println("NRF24 OK.");
   

   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(config.nrf_RXTXchannel); // set the channel
   radio->openReadingPipe(1,config.nrf_RXTXaddress); // set network address
   radio->startListening(); // start listening for data
   Serial.println("NRF24 init OK.");
   nrf24.radioOK = true;

   for(;;){
      if(nrf24.radioOK == false) return;  //return if nrf24 is not initialized

      if ( radio->available() ) {
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, HIGH);

         radio->read( nrf24.payload, sizeof(nrf24.payload) ); // get data packet from radio 
         for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
            uint16_t _channel = NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ;
            // uint8_t _value = nrf24.payload[i+2];
            Dmxwire.write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]); // parse radio data into dmx data array
         } 
         // Serial.println(NRF24_BYTES_PER_PACKET * nrf24.payload[0] ,nrf24.payload[2]);

         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
         
      } 

      xSemaphoreTake(sync_config, portMAX_DELAY);
      uint8_t _iomode = config.ioMode;
      xSemaphoreGive(sync_config);
      if(_iomode != DMXBOARD_MODE_RX_NRF24){ //if ioMode is not longer RX NRF24, delete task
         vTaskDelete(NULL);
      }
      delay(5);
   }
}

void DMXWire::nrf24RX_toDmx512(void*pvParameters){
   if(nrf24.radioOK == false) return;  //return if nrf24 is not initialized

   if ( radio->available() ) {
      if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, HIGH);

      radio->read( nrf24.payload, sizeof(nrf24.payload) ); // get data packet from radio 
      for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
         uint16_t _channel = NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ;
         DMX::Write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]);
      } 

      if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
      
   } 

   xSemaphoreTake(sync_config, portMAX_DELAY);
   uint8_t _iomode = config.ioMode;
   xSemaphoreGive(sync_config);
   if(_iomode != DMXBOARD_MODE_NRF24TODMX512){ //if ioMode is not longer RX NRF24, delete task
      vTaskDelete(NULL);
   }
}

void DMXWire::serialhandler(){   //
   if(Serial.available()){
      String _cmd = Serial.readStringUntil('\n');  //read line
      // Serial.println(_cmd);
      if(_cmd.indexOf(".") < 0){
         Serial.println("wrong cmd");
         return;
      }
      
      String _cmd0 = _cmd.substring(0, _cmd.indexOf("."));
      String _cmd1 = _cmd.substring(_cmd.indexOf(".")+1, _cmd.length());

      xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
      dmxwire_settings_t _cfg = config;
      xSemaphoreGive(sync_config);
      // return;

      // Serial.printf("CMD0:%s, CMD1:%s\n",_cmd0.c_str(), _cmd1.c_str());
      int cmd0 = _cmd0.toInt();
      int cmd1 = _cmd1.toInt();
      if(cmd0 == 0){
         if(cmd1 == 0){ //write to EEPROM
            Serial.println("write things to eeprom...");
            Dmxwire.writeConfig();
         }else if(cmd1 == -1){   //restart
            Serial.println("restart device");
            ESP.restart();
         }else if(cmd1 == -2){   //hardreset device
            Serial.println("hardreset Device...");
            dmxwire_settings_t _newConfig;
            xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
            config = _newConfig;
            xSemaphoreGive(sync_config);
            Dmxwire.writeConfig();
            Serial.println("restart device");
            ESP.restart();
         }
      }else if(cmd0 == 1){ //io Mode
         if(cmd1 == -1){   //ioMode idle
            _cfg.ioMode = DMXBOARD_MODE_OFF;
            Serial.printf("ioMode: idle(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 0){ //ioMode RX dmx512
            _cfg.ioMode = DMXBOARD_MODE_TX_DMX512;
            Serial.printf("ioMode: RX dmx512(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 1){ //ioMode RX NRF24
            _cfg.ioMode = DMXBOARD_MODE_TX_NRF24;
            Serial.printf("ioMode: rx nrf24(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 2){ //ioMode TX dmx512
            _cfg.ioMode = DMXBOARD_MODE_RX_DMX512;
            Serial.printf("ioMode: tx dmx512(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 3){ //ioMode TX NRF24
            _cfg.ioMode = DMXBOARD_MODE_RX_NRF24;
            Serial.printf("ioMode: tx nrf24(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 4){ //ioMode DMX to NRF24
            _cfg.ioMode = DMXBOARD_MODE_DMX512TONRF24;
            Serial.printf("ioMode: dmx512 to nrf24(%u)\n", _cfg.ioMode);
         }else if(cmd1 == 5){ //ioMode DMX to NRF24
            _cfg.ioMode = DMXBOARD_MODE_NRF24TODMX512;
            Serial.printf("ioMode: nrf24 to dmx512(%u)\n", _cfg.ioMode);
         }
      }else if(cmd0 == 4){   //NRF24 get
         if(cmd1 >= 0 && cmd1 <= 255){ //get Noise on channel x
            Serial.printf("noise: %i\n", random(0,255));
         }else if(cmd1 == -1){
            Serial.printf("rx/tx channel:%i", _cfg.nrf_RXTXchannel);
         }
      }else if(cmd0 == 5){ //NRF24 set
         if(cmd1 >= 0 && cmd1 <= 255){ //get Noise on channel x
            _cfg.nrf_RXTXchannel = cmd1;
            Serial.printf("rx/tx channel:%i", _cfg.nrf_RXTXchannel);
         }else if(cmd1 == -1){
            Serial.println("rx/tx channel on Automode");
         }
      }

      xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
      config = _cfg;
      xSemaphoreGive(sync_config);
      delay(2);
   }
}


void DMXWire::preferencesInit(){
   preferences = new Preferences();
   preferences->begin("dmxwire_config", false);
   Serial.println("preferences initialized");

}

void DMXWire::readConfig(){

   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety

   config.ioMode = preferences->getUChar("ioMode", DMXBOARD_MODE_TX_DMX512);
   config.ledRxMode = preferences->getUChar("ledRxMode", DMXWIRE_LED_WIRE);
   config.ledRxpin = preferences->getInt("ledRxPin", LED_BUILTIN);
   config.ledTxMode = preferences->getUChar("ledTxMode",DMXWIRE_LED_DMX512);
   config.ledTxpin = preferences->getInt("ledTxPin", -1);
   config.nrf_RXTXaddress = preferences->getULong64("nrf_RXTXaddress",0xF0F0F0F0F0LL );
   config.nrf_RXTXaddress = preferences->getUChar("nrf_RXTXchannel", 0);
   config.timeout_dmx512_ms = preferences->getULong("timeout_dmx512_ms", 500);
   config.timeout_nrf24_ms = preferences->getULong("timeout_nrf24_ms", 500);
   config.timeout_wire_ms = preferences->getULong("timeout_wire_ms", 500);


   // Serial.printf("ioMode:%u\n",config.ioMode);
   // Serial.printf("ledRxpin:%i\n",config.ledRxpin);
   // Serial.printf("ledTxpin:%i\n",config.ledTxpin);
   // Serial.printf("tx framerate:%u\n",config.txFramerate_ms);
   // Serial.printf("ledRxMode:%u\n",config.ledRxMode);
   // Serial.printf("ledTxMode:%u\n",config.ledTxMode);
   // Serial.printf("nrf_RXTXaddress:%u\n",config.nrf_RXTXaddress);
   // Serial.printf("nrf_RXTXchannel:%u\n",config.nrf_RXTXchannel);
   // Serial.printf("timeout_wire_ms:%lu\n",config.timeout_wire_ms);
   // Serial.printf("timeout_dmx512_ms:%lu\n",config.timeout_dmx512_ms);
   // Serial.printf("timeout_nrf24_ms:%lu\n",config.timeout_nrf24_ms);
   // Serial.println("=======================");


   xSemaphoreGive(sync_config);
   
}

void DMXWire::writeConfig(){
   Serial.println("write...");
   
   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   Serial.printf("set ioMode:%u size of config: %u\n", config.ioMode, sizeof(config));

   preferences->putUChar("ioMode", config.ioMode);
   preferences->putUChar("ledRxMode", config.ledRxMode);
   preferences->getInt("ledRxPin", config.ledRxpin);
   preferences->putUChar("ledTxMode", config.ledTxMode);
   preferences->getInt("ledTxPin", config.ledTxpin);
   preferences->getULong64("nrf_RXTXaddress", config.nrf_RXTXaddress );
   preferences->getUChar("nrf_RXTXchannel", config.nrf_RXTXaddress);
   preferences->getULong("timeout_dmx512_ms", config.timeout_dmx512_ms);
   preferences->getULong("timeout_nrf24_ms", config.timeout_nrf24_ms);
   preferences->getULong("timeout_wire_ms", config.timeout_wire_ms);

   xSemaphoreGive(sync_config);

   Dmxwire.readConfig();
   
}