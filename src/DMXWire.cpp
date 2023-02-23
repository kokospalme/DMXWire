/*
 * DMXWire.cpp
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 * 
 * ToDo: xTaskHandler on tasks + task delete von Außen, nicht von innen!
 */

#include "DMXWire.h"

uint8_t DMXWire::packets[DMXWIRE_PACKETS][DMXWIRE_BYTES_PER_PACKET];
uint8_t DMXWire::packetNo = 0;
uint8_t DMXWire::slaveAddress = 1;	//slave's address
int DMXWire::packetBusy =  DMXWIRE_NOTBUSY;
unsigned long DMXWire::duration_wire = 0;
unsigned long DMXWire::timestamp_wire = 0;
unsigned long DMXWire::timestamp_dmx512 = 0;
unsigned long DMXWire::timestamp_nrf24 = 0;
dmxwire_settings_t DMXWire::config;
SemaphoreHandle_t DMXWire::sync_dmx;
SemaphoreHandle_t DMXWire::sync_config;
dmxwire_request_t DMXWire::request;
dmxwire_status_t DMXWire::slave_status;
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

void DMXWire::beginStandalone(){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   Serial.println("read config from preferences:");
   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio
   Dmxwire.preferencesInit();
   Dmxwire.readConfig();

   config.nrf_RXTXaddress = 0xF0F0F0F0F0LL;
   config.ledRxpin = LEDRX_PIN;  //overwrite pins
   config.ledTxpin = LEDTX_PIN;

   switchIomode();

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
	return duration_wire;
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


void DMXWire::slave_dmx512rx_task(void*pvParameters){
   uint8_t _iomode = 0;
   uint8_t _ledRxMode = 0;
   int _ledRxPin = 0;
   bool _healthy = 0;
   
   for(;;){
      xSemaphoreTake(sync_config, portMAX_DELAY);
      _iomode = config.ioMode;
      _ledRxMode = config.ledRxMode;
      _ledRxPin = config.ledRxpin;
      xSemaphoreGive(sync_config);

      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //dmx healthy?
      slave_status.dmx512_healthy = (bool) DMX::IsHealthy();
      _healthy = slave_status.dmx512_healthy;
      xSemaphoreGive(sync_dmx);


      if(_healthy){ //
         if(_ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(_ledRxPin, HIGH);
         // uint8_t _buffer[512];
         // DMX::ReadAll(_buffer, 1, 512);

         for(int i = 1; i <= 512 ;i++){   //write from DMX to Wire buffer
            // Dmxwire.write(i, _buffer[i-1]);
            Dmxwire.write(i, DMX::Read(i));
         }

         if(_iomode != DMXBOARD_MODE_RX_DMX512){ //if ioMode is not longer TX NRF24, delete task
         Serial.println("delete DMX512 RX task");
            if(_ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(_ledRxPin, LOW);
            vTaskDelete(NULL);
         }
      }else{
         if(_ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(_ledRxPin, LOW);
      }


      delay(20);
   }
}

void DMXWire::nrf24tx_task(void*pvParameters) { //transmit via NRF24
   // radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio

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
   radio->openWritingPipe(config.nrf_RXTXaddress); // set network address
   radio->stopListening(); // start talking !
   Serial.println("NRF24 init OK.");
   nrf24.radioOK = true;

   for(;;){
      if(nrf24.radioOK == false) return;  //return if nrf24 is not initialized

      for (uint8_t group = 0; group < NRF24_MAXGROUPS; group++) { // send groups of DMX data, 16 bytes at a time
         uint16_t group_ptr = group * NRF24_BYTES_PER_PACKET; // create group pointer for array
         if (millis() > timestamp_nrf24 + config.timeout_nrf24_ms) { // allow ALL radio data (full DMX array) to be send once per second, regardless of changes
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
            timestamp_nrf24 = millis(); // reset timestamp
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
         Serial.println("delete NRF24 TX task");
         digitalWrite(config.ledTxpin, LOW);
         switchIomode();
         vTaskDelete(NULL);
      }
      delay(3);
   }
} 

void DMXWire::nrf24rx_task(void*pvParameters){
   // radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio

   nrf24.radioOK = false;
   Serial.println("NRF24 initializing (RX)...");
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
      if ( radio->available() ) {
         timestamp_nrf24 = millis();
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, HIGH);

         radio->read( nrf24.payload, sizeof(nrf24.payload) ); // get data packet from radio 
         for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
            uint16_t _channel = NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ;
            // uint8_t _value = nrf24.payload[i+2];
            Dmxwire.write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]); // parse radio data into dmx data array
         } 
         // Serial.println(NRF24_BYTES_PER_PACKET * nrf24.payload[0] ,nrf24.payload[2]);

      } 

      xSemaphoreTake(sync_config, portMAX_DELAY);
      uint8_t _iomode = config.ioMode;
      unsigned long _timestamp = config.timeout_nrf24_ms;
      xSemaphoreGive(sync_config);
      if(_iomode != DMXBOARD_MODE_RX_NRF24){ //if ioMode is not longer RX NRF24, delete task
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
         Serial.println("delete NRF24 RX Task");
         digitalWrite(config.ledRxpin, LOW);
         switchIomode();
         vTaskDelete(NULL);
      }
      if(millis() > timestamp_nrf24 + _timestamp){ //LED handling
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
      }

   }
}

void DMXWire::nrf24rx_toDmx512(void*pvParameters){
   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio

   nrf24.radioOK = false;
   Serial.println("NRF24 initializing (RX)...");
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
         timestamp_nrf24 = millis();
         timestamp_dmx512 = millis();
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, HIGH);
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, HIGH);

         radio->read( nrf24.payload, sizeof(nrf24.payload) ); // get data packet from radio 
         for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
            uint16_t _channel = NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ;
            // uint8_t _value = nrf24.payload[i+2];
            Dmxwire.write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]); // parse radio data into dmx data array
            DMX::Write(NRF24_BYTES_PER_PACKET * nrf24.payload[0] + i ,nrf24.payload[i+2]);
         } 
         // Serial.println(NRF24_BYTES_PER_PACKET * nrf24.payload[0] ,nrf24.payload[2]);
         
      } 

      xSemaphoreTake(sync_config, portMAX_DELAY);
      uint8_t _iomode = config.ioMode;
      unsigned long _timestamp = config.timeout_nrf24_ms;
      xSemaphoreGive(sync_config);

      if(_iomode != DMXBOARD_MODE_NRF24TODMX512){ //if ioMode is not longer NRF24 to dmx512, delete task
         Serial.println("delete NRF24 to dmx512 Task");
         radio->stopListening();
         digitalWrite(config.ledRxpin, LOW);
         digitalWrite(config.ledTxpin, LOW);
         switchIomode();
         vTaskDelete(NULL);
      }
      if(millis() > timestamp_nrf24 + _timestamp){ //LED handling
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
      }
   }
}

void DMXWire::serialhandlerSlave(){   //
   if(Serial.available()){
      String _cmd = Serial.readStringUntil('\n');  //read line
      // Serial.println(_cmd);
      if(_cmd.indexOf(".") < 0){
         Serial.println("wrong cmd");
         return;
      }
      String _cmd0 = _cmd.substring(0, _cmd.indexOf("."));
      String _cmd1 = _cmd.substring(_cmd.indexOf(".")+1, _cmd.length());

      uint16_t cmd0 = _cmd0.toInt();
      uint16_t cmd1 = _cmd1.toInt();

      settingshandler(cmd0, cmd1);
   }
}

void DMXWire::serialhandlerMaster(){   //
   // if(Serial.available()){
   //    String _cmd = Serial.readStringUntil('\n');  //read line
   //    // Serial.println(_cmd);
   //    if(_cmd.indexOf(".") < 0){
   //       Serial.println("wrong cmd");
   //       return;
   //    }
   //    String _cmd0 = _cmd.substring(0, _cmd.indexOf("."));
   //    String _cmd1 = _cmd.substring(_cmd.indexOf(".")+1, _cmd.length());

   //    uint16_t cmd0 = _cmd0.toInt();
   //    uint16_t cmd1 = _cmd1.toInt();

   //    if(cmd0 == SETTINGID_SAVE_TO_EEPROM){ //write to EEPROM
   //       Serial.println("write things to eeprom...");
   //       Dmxwire.writeConfig();
   //       sendAck(0);
   //    }else if(cmd0 == SETTINGID_RESTART_DEVICE){   //restart
   //       Serial.println("restart device");
   //       sendAck(0);
   //       ESP.restart();
   //    }else if(cmd0 == SETTINGID_HARDRESET){   //hardreset device
   //       Serial.println("hardreset Device...");
   //       dmxwire_settings_t _newConfig;
   //       xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   //       config = _newConfig;
   //       xSemaphoreGive(sync_config);
   //       Dmxwire.writeConfig();
   //       sendAck((uint16_t)_cfg.ioMode);
   //       Serial.println("restart device");
   //       ESP.restart();
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_IDLE){   //ioMode idle
   //       _cfg.ioMode = DMXBOARD_MODE_OFF;
   //       Serial.printf("set ioMode: idle(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_TX_DMX512){ //ioMode TX dmx512
   //       _cfg.ioMode = DMXBOARD_MODE_TX_DMX512;
   //       Serial.printf("set ioMode: RX dmx512(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_TX_NRF24){ //ioMode TX NRF24
   //       _cfg.ioMode = DMXBOARD_MODE_TX_NRF24;
   //       Serial.printf("set ioMode: tx nrf24(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_TX_NRF24){ //ioMode RX dmx512
   //       _cfg.ioMode = DMXBOARD_MODE_RX_DMX512;
   //       Serial.printf("set ioMode: tx dmx512(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_RX_NRF24){ //ioMode RX NRF24
   //       _cfg.ioMode = DMXBOARD_MODE_RX_NRF24;
   //       Serial.printf("set ioMode: rx nrf24(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_DMX512TONRF24){ //ioMode DMX to NRF24
   //       _cfg.ioMode = DMXBOARD_MODE_DMX512TONRF24;
   //       Serial.printf("set ioMode: dmx512 to nrf24(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_SET_IOMODE_NRF24TODMX512){ //ioMode DMX to NRF24
   //       _cfg.ioMode = DMXBOARD_MODE_NRF24TODMX512;
   //       Serial.printf("set ioMode: nrf24 to dmx512(%u)\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);
   //    }else if(cmd0 == SETTINGID_GET_IOMODE){ //get io Mode
   //       Serial.printf("get ioMode:%u\n", _cfg.ioMode);
   //       sendAck((uint16_t)_cfg.ioMode);


   //    }else if(cmd0 == SETTINGID_GET_DMX512_TIMEOUT){ //get dmx512 timeout
   //       Serial.printf("get dmx512 timeout:%u\n", _cfg.timeout_dmx512_ms);
   //       sendAck((uint16_t)_cfg.timeout_dmx512_ms);

   //    }else if(cmd0 == SETTINGID_SET_DMX512_TIMEOUT){ //set dmx512 timeout
   //       _cfg.timeout_dmx512_ms = (unsigned long) cmd1;
   //       Serial.printf("set dmx512 timeout:%u\n", _cfg.timeout_dmx512_ms);
   //       sendAck((uint16_t)_cfg.timeout_dmx512_ms);

   //    }else if(cmd0 == SETTINGID_GET_DMX512_FPS){ //get dmx512 transmit FPS
   //       Serial.printf("get dmx512 timeout:%u\n", _cfg.txFramerate_ms);
   //       sendAck((uint16_t)_cfg.txFramerate_ms);

   //    }else if(cmd0 == SETTINGID_SET_DMX512_FPS){ //set dmx512 transmit FPS
   //       _cfg.txFramerate_ms = (uint8_t) cmd1;
   //       Serial.printf("set dmx512 timeout:%u\n", _cfg.txFramerate_ms);
   //       sendAck((uint16_t)_cfg.txFramerate_ms);


   //    }else if(cmd0 == SETTINGID_GET_NRF24_TIMEOUT){ //get nrf24 timeout
   //       Serial.printf("get nrf24 timeout:%u\n", _cfg.timeout_nrf24_ms);
   //       sendAck((uint16_t)_cfg.timeout_nrf24_ms);

   //    }else if(cmd0 == SETTINGID_SET_NRF24_TIMEOUT){ //get nrf24 timeout
   //       _cfg.timeout_nrf24_ms = (unsigned long) cmd1;
   //       Serial.printf("set nrf24 timeout:%u\n", _cfg.timeout_nrf24_ms);
   //       sendAck((uint16_t)_cfg.timeout_nrf24_ms);

   //    }else if(cmd0 == SETTINGID_GET_NRF24_NOISE){ //get nrf24 timeout
   //       uint8_t _noise = random(0,255);  //ToDo: implement getNoise
   //       Serial.printf("get nrf24 noise:%u\n", _noise);
   //       sendAck((uint16_t)_noise);
   //    }else if(cmd0 == SETTINGID_SET_NRF24_CHANNEL){ //get nrf24 timeout
   //       if(cmd1 == SETTINGID_SET_NRF24_CHANNEL_AUTOMODE){
   //          _cfg.nrf_RXTXchannel = 0;
   //          Serial.println("set nrf24 channel: automode");
   //       }else{
   //          _cfg.nrf_RXTXchannel = (uint8_t) cmd1;
   //          Serial.printf("set nrf24 channel:%u\n", _cfg.nrf_RXTXchannel);
   //       }
   //       sendAck((uint16_t)_cfg.nrf_RXTXchannel);
   //    }else if(cmd0 == SETTINGID_GET_NRF24_CHANNEL){ //get nrf24 timeout
   //       Serial.printf("get nrf24 channel:%u\n", _cfg.nrf_RXTXchannel);
   //       sendAck((uint16_t)_cfg.nrf_RXTXchannel);
   //    }
   // }
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


   Serial.printf("ioMode:%u\n",config.ioMode);
   Serial.printf("ledRxpin:%i\n",config.ledRxpin);
   Serial.printf("ledTxpin:%i\n",config.ledTxpin);
   Serial.printf("tx framerate:%u\n",config.txFramerate_ms);
   Serial.printf("ledRxMode:%u\n",config.ledRxMode);
   Serial.printf("ledTxMode:%u\n",config.ledTxMode);
   Serial.printf("nrf_RXTXaddress:%u\n",config.nrf_RXTXaddress);
   Serial.printf("nrf_RXTXchannel:%u\n",config.nrf_RXTXchannel);
   Serial.printf("timeout_wire_ms:%lu\n",config.timeout_wire_ms);
   Serial.printf("timeout_dmx512_ms:%lu\n",config.timeout_dmx512_ms);
   Serial.printf("timeout_nrf24_ms:%lu\n",config.timeout_nrf24_ms);
   Serial.println("=======================");


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

   // Dmxwire.readConfig();
   // ESP.restart(); //ToDo: restart nötig oder nicht?!
   
}

void DMXWire::switchIomode(){

#define IOMODESWITCH_DELAY 200

   switch(config.ioMode){
      case DMXBOARD_MODE_OFF:
      return;
      break;
      case DMXBOARD_MODE_TX_DMX512:
         delay(IOMODESWITCH_DELAY);
         Dmxwire.setLedTx(DMXWIRE_LED_DMX512);
         Dmxwire.setLedRx(DMXWIRE_LED_WIRE);
         DMX::Initialize(output);
         Serial.println("TX dmx512 initialized");
      break;
      case DMXBOARD_MODE_TX_NRF24:
         delay(IOMODESWITCH_DELAY);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("init MODE_TX_NRF24");
         DMX::Initialize(input);
         xTaskCreatePinnedToCore(DMXWire::nrf24tx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, NRF24_CORE);
      break;
      case DMXBOARD_MODE_RX_DMX512:
         delay(IOMODESWITCH_DELAY);
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init MODE_RX_DMX512");
         DMX::Initialize(input);
         xTaskCreatePinnedToCore(DMXWire::slave_dmx512rx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, DMX512_CORE);
      break;
      case DMXBOARD_MODE_RX_NRF24:
         delay(IOMODESWITCH_DELAY);
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init MODE_RX_NRF24");
         xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
      break;
      case DMXBOARD_MODE_DMX512TONRF24:
         delay(IOMODESWITCH_DELAY);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init DMX512TONRF24");
         DMX::Initialize(input);
         // xTaskCreatePinnedToCore(DMXWire::nrf24tx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, NRF24_CORE);
      break;
      case DMXBOARD_MODE_NRF24TODMX512:
         delay(IOMODESWITCH_DELAY);
         setLedTx(DMXWIRE_LED_DMX512);
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init NRF24TODMX512");
         DMX::Initialize(output);
         // xTaskCreatePinnedToCore(DMXWire::nrf24rx_toDmx512, "nrf24_rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
      break;
      default:break;
   }
}