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
dmxwire_hardware_t DMXWire::hardwareCfg;
dmxwire_settings_t DMXWire::config;
SemaphoreHandle_t DMXWire::sync_dmx;
SemaphoreHandle_t DMXWire::sync_config;
SemaphoreHandle_t DMXWire::sync_wire;
dmxwire_request_t DMXWire::request;
dmxwire_status_t DMXWire::slave_status;
nrf24Data_t DMXWire::nrf24;
Preferences *DMXWire::preferences;
RF24 *DMXWire::radio;
bool DMXWire::rf24Initialized = false;
bool DMXWire::dmxInitialized = false;
TaskHandle_t DMXWire::xSlave_dmx512rx_taskhandler;
TaskHandle_t DMXWire::xNrf24tx_taskhandler;
TaskHandle_t DMXWire::xNrf24rx_taskhandler;
TaskHandle_t DMXWire::xNrf24rx_to_dmx512_taskhandler;
TaskHandle_t DMXWire::xDmx512_to_nrf24_taskhandler;
bool DMXWire::usePreferences = false;

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

/*
set the clock frequency for I2C communication/Wire
*/
void DMXWire::setClock(uint32_t frequency){
	Wire.setClock(frequency);
}

/*
set pin and mode for RX-indicating LED (default: builtin LED, receive over WIre)
*/
void DMXWire::setLedRx(int pin, uint8_t mode){	//set pin and mode for ledRx
	config.ledRxpin = pin;
	config.ledRxMode = mode;
	pinMode(config.ledRxpin, OUTPUT);
	digitalWrite(config.ledRxpin, LOW);
}

/*
set mode for RX-indicating LED (default: RX on Wire)
*/
void DMXWire::setLedRx(uint8_t mode){	//set mode for ledRx
	config.ledRxMode = mode;
	pinMode(config.ledRxpin, OUTPUT);
	digitalWrite(config.ledRxpin, LOW);
}

/*
set pin and mode for TX-indicating LED (default: no LED, tramsmit over DMX512)
*/
void DMXWire::setLedTx(int pin, uint8_t mode){	//set pin and mode for ledTx
	config.ledTxpin = pin;
	config.ledTxMode = mode;
	pinMode(config.ledTxpin, OUTPUT);
	digitalWrite(config.ledTxpin, LOW);
}

/*
set pin and mode for TX-indicating LED (default: Rtransmit over DMX512)
*/
void DMXWire::setLedTx(uint8_t mode){	//set mode for ledRx
	config.ledTxMode = mode;
	pinMode(config.ledTxpin, OUTPUT);
	digitalWrite(config.ledTxpin, LOW);
}

/*
set the IOmode and store it to preferences
*/
void DMXWire::setIomode(uint8_t mode){
   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   config.ioMode = DMXBOARD_MODE_OFF;
   switchIomode();
   xSemaphoreGive(sync_config);
   Serial.println("MODE: OFF (tasks deleted)");
   // while(!Serial.available());
   delay(100);

   Serial.printf("dmxwire iomode: %i\n", mode);

   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   if(mode != config.ioMode){
      config.ioMode = mode;
   }

   Serial.println("iomode locally written.");
   xSemaphoreGive(sync_config);

   // while(!Serial.available());
   delay(100);
   
   
   preferences->putUChar("ioMode", config.ioMode);
   Serial.println("iomode stored.");

   // while(!Serial.available());
   delay(100);
   switchIomode();


   

   
}

/*
set the datapipe-address through which the device is sending/receiving Data over NRF24 module and store it to preferences
*/
void DMXWire::setRxtxpipe(uint64_t rxtxAddress){

   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   config.nrf_RXTXaddress = rxtxAddress;
   if(usePreferences)preferences->putULong64("nrf_RXTXaddress", rxtxAddress);
   xSemaphoreGive(sync_config);
   
}


/*

*/
void DMXWire::setHardware(dmxwire_hardware_t hardware){
   hardwareCfg = hardware;
}


/*
set dmxwire config
*/
void DMXWire::setConfig(dmxwire_settings_t cfg){
   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   config = cfg;
   xSemaphoreGive(sync_config);
}

dmxwire_settings_t DMXWire::getConfig(){

   dmxwire_settings_t _cfg;
   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   _cfg = config;
   xSemaphoreGive(sync_config);
   return _cfg;
}


/*
Begin as Standalone-device without I2C connection
- switchIomode() has to be executed manually afterwards
*/
void DMXWire::beginStandalone(bool usePreferences){

   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   DMXWire::usePreferences = usePreferences;
   if(usePreferences) Serial.println("read config from preferences:");
   radio = new RF24(hardwareCfg.nrf_ce, hardwareCfg.nrf_cs);  //init radio
   radio->setPALevel(RF24_PA_MAX);
   if(usePreferences) Dmxwire.preferencesInit();
   if(usePreferences) Dmxwire.readConfig();


   // config.nrf_RXTXaddress = RXTX_ADDRESS;
   // config.ledRxpin = LEDRX_PIN;  //overwrite pins
   // config.ledTxpin = LEDTX_PIN;
   if(config.ledRxpin > -1)pinMode(config.ledRxpin, OUTPUT);
   if(config.ledRxpin > -1)pinMode(config.ledTxpin, OUTPUT);

   Dmxwire.initNRF24();

   // switchIomode();

}

/*
Inititalize NRF24 module
*/
void DMXWire::initNRF24(){

   nrf24.radioOK = false;
   Serial.print("NRF24 initializing... ");
   if(!radio->begin()){
      Serial.println("ERROR: nrf24 module is not responding.");
   }else{
      Serial.println("NRF24 Module OK.");
   }
   
   rf24Initialized = true;

   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(config.nrf_RXTXchannel); // set the channel
   radio->openReadingPipe(1,config.nrf_RXTXaddress); // set network address
   radio->startListening();
   radio->stopListening(); //go to standby mode
   Serial.println("NRF24 init OK.");
   nrf24.radioOK = true;


}


/*
Begin as Slave that receives data from Master
*/
void DMXWire::beginSlaveRX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   Serial.println("begin Slave RX");
   usePreferences = true;
	Wire.begin(slaveaddress, sda, scl,clock);
	Wire.onReceive(DMXWire::slaveRXcallback); // register event
	DMXWire::slaveAddress = slaveaddress;
}

/*
Read values from DMX array
*/
uint8_t DMXWire::read(uint16_t channel){
	if(channel < 1 || channel > 512) return 0;	//dmx borders

	uint16_t _packetNo = (channel-1) / DMXWIRE_CHANNEL_PER_PACKET;
	uint16_t _byteNo = (channel - 1) - _packetNo * DMXWIRE_CHANNEL_PER_PACKET;

   xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
   uint8_t _value = packets[_packetNo][DMXWIRE_HEAD + _byteNo];
   xSemaphoreGive(sync_dmx);

	return _value;
   
}

void DMXWire::readAll(uint8_t *data){
   // xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
   // memcpy(data, (uint8_t *)dmx_data, 512);
   // xSemaphoreGive(sync_dmx);
}

/*
Returns duration for getting all 512 bytes in ms
*/
unsigned long DMXWire::getDuration(){
	return duration_wire;
}


/*
Set timeout for I2C/Wire in ms
*/
void DMXWire::setTimout_wire(unsigned long timeout_ms){
	config.timeout_wire_ms = timeout_ms;
}

/*
Set timeout for dmx512 in ms
*/
void DMXWire::setTimout_dmx512(unsigned long timeout_ms){
	config.timeout_dmx512_ms = timeout_ms;
}

/*
Set timeout for nrf24 in ms
*/
void DMXWire::setTimout_nrf24(unsigned long timeout_ms){
	config.timeout_nrf24_ms = timeout_ms;
}


/*
Returns true if last msg from I2C/Wire has timed out
*/
bool DMXWire::getTimeout_wire(){
	if(millis() > timestamp_wire + config.timeout_wire_ms){
		if(config.ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

/*
Returns true if last msg from dmx512 has timed out
*/
bool DMXWire::getTimeout_dmx512(){
	if(millis() > timestamp_dmx512 + config.timeout_dmx512_ms){
		if(config.ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

/*
Returns true if last msg from nrf24 has timed out
*/
bool DMXWire::getTimeout_nrf24(){
	if(millis() > timestamp_nrf24 + config.timeout_nrf24_ms){
		if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
		if(config.ledTxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledTxpin, LOW);
		return true;
	}else{	//no timeout
		return false;
	}
}

/*
Write values to DMX array
*/
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





/*
Handles serial commands at Slave-MCU
*/
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


/*
Handles serial commands at Master-MCU
*/
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

/*
Initialize preferences object
*/
void DMXWire::preferencesInit(){
   if(!usePreferences)return;
   preferences = new Preferences();
   if(preferences->begin("dmxwire_config", false))Serial.println("preferences initialized");
   else Serial.println("preferences initialized failed.");

}


/*
Read config from preferences and write it to config object
*/
void DMXWire::readConfig(){
   if(!usePreferences)return;

   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   config.ioMode = preferences->getUChar("ioMode", DMXBOARD_MODE_OFF);
   config.ledRxMode = preferences->getUChar("ledRxMode", DMXWIRE_LED_WIRE);
   config.ledRxpin = preferences->getInt("ledRxPin", -1);
   config.ledTxMode = preferences->getUChar("ledTxMode",DMXWIRE_LED_DMX512);
   config.ledTxpin = preferences->getInt("ledTxPin", -1);
   config.nrf_RXTXaddress = preferences->getULong64("nrf_RXTXaddress",0xF0F0F0F0F0LL );
   config.nrf_RXTXchannel = preferences->getUChar("nrf_RXTXchannel", 0);
   config.timeout_dmx512_ms = preferences->getULong("timeout_dmx512_ms", 500);
   config.timeout_nrf24_ms = preferences->getULong("timeout_nrf24_ms", 500);
   config.timeout_wire_ms = preferences->getULong("timeout_wire_ms", 500);


   Serial.println("=== CONFIG READ: ===");
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
   if(!usePreferences)return;
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

/*
Switch to another IOmode (delete old task, start one)
*/
void DMXWire::switchIomode(){
   if(config.ledRxpin >=0) digitalWrite(config.ledRxpin, LOW);
   if(config.ledTxpin >=0) digitalWrite(config.ledTxpin, LOW);


   if( xSlave_dmx512rx_taskhandler != NULL ){   //stop slave_dmx512rx_task
      vTaskDelete( xSlave_dmx512rx_taskhandler );
      Serial.println("delete xSlave_dmx512rx_task");
      delay(500);
   }
   if( xNrf24tx_taskhandler != NULL ){   //stop nrf24tx_task
      vTaskDelete( xNrf24tx_taskhandler );
      // radio->closeReadingPipe(1); // set network address
      digitalWrite(config.ledTxpin, LOW);
      Serial.println("delete xSlave_dmx512rx_task");
      delay(500);
   }
   if( xNrf24rx_taskhandler != NULL ){   //stop nrf24rx_task
      vTaskDelete( xNrf24rx_taskhandler );
      // radio->stopListening();
      // radio->closeReadingPipe(1);
      Serial.println("delete xNrf24rx_task");
      delay(500);
   }
   if( xNrf24rx_to_dmx512_taskhandler != NULL ){   //stop nrf24rx_toDmx512_task
      vTaskDelete( xNrf24rx_to_dmx512_taskhandler );
      // radio->stopListening();
      // radio->closeReadingPipe(1);
      Serial.println("delete xNrf24rx_toDmx512_task");
      delay(500);
   }
   if( xDmx512_to_nrf24_taskhandler != NULL ){   //stop nrf24rx_toDmx512_task
      vTaskDelete( xDmx512_to_nrf24_taskhandler );
      // radio->closeReadingPipe(1); // set network address
      Serial.println("delete dmx512_to_nrf24_task");
      delay(500);
   }


   switch(config.ioMode){
      case DMXBOARD_MODE_OFF:
      return;
      break;
      case DMXBOARD_MODE_TX_DMX512:
         Dmxwire.setLedTx(DMXWIRE_LED_DMX512);
         Dmxwire.setLedRx(DMXWIRE_LED_WIRE);
         if(dmxInitialized) DMX::changeDirection(output);
         else DMX::Initialize(output);
         dmxInitialized = true;
         Serial.println("init MODE_TX_DMX512");
      break;
      case DMXBOARD_MODE_TX_NRF24:
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("init MODE_TX_NRF24. start nrf24tx_task");
         // if(dmxInitialized) DMX::changeDirection(input);
         // else DMX::Initialize(input);
         // dmxInitialized = true;
         xTaskCreatePinnedToCore(DMXWire::nrf24tx_task, "nrf24_tx_task", 1024, NULL, MODE_TX_NRF24_PRIO, &DMXWire::xNrf24tx_taskhandler, NRF24_CORE);
      break;
      case DMXBOARD_MODE_RX_DMX512:
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init MODE_RX_DMX512. start slave_dmx512rx_task");
         if(dmxInitialized) DMX::changeDirection(input);
         else DMX::Initialize(input);
         dmxInitialized = true;
         xTaskCreatePinnedToCore(DMXWire::slave_dmx512rx_task, "slave_dmx512rx_task", 1024, NULL, 1, &DMXWire::xSlave_dmx512rx_taskhandler, DMX512_CORE);
      break;
      case DMXBOARD_MODE_RX_NRF24:
         if(config.ledTxpin >= 0)setLedTx(DMXWIRE_LED_WIRE);
         if(config.ledRxpin >= 0)setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init MODE_RX_NRF24. start nrf24rx_task");
         xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24rx_task", 1024, NULL, MODE_RX_NRF24_PRIO, &DMXWire::xNrf24rx_taskhandler, NRF24_CORE);
      break;
      case DMXBOARD_MODE_DMX512TONRF24:
         if(config.ledTxpin >= 0)setLedTx(DMXWIRE_LED_NRF24);
         if(config.ledRxpin >= 0)setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init DMX512TONRF24. start dmx512_to_nrf24_task");
         if(dmxInitialized) DMX::changeDirection(input);
         else DMX::Initialize(input);
         dmxInitialized = true;
         xTaskCreatePinnedToCore(DMXWire::dmx512_to_nrf24_task, "dmx512_to_nrf24_task", MODE_DMX512TONRF24_STACKSIZE, NULL, DMX512TONRF24_PRIO, &DMXWire::xDmx512_to_nrf24_taskhandler, DMX512_CORE);
      break;
      case DMXBOARD_MODE_NRF24TODMX512:
         if(config.ledTxpin >= 0)setLedTx(DMXWIRE_LED_DMX512);
         if(config.ledRxpin >= 0)setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init NRF24TODMX512. start nrf24rx_toDmx512_task");
         if(dmxInitialized) DMX::changeDirection(output);
         else DMX::Initialize(output);
         dmxInitialized = true;
         xTaskCreatePinnedToCore(DMXWire::nrf24rx_toDmx512_task, "nrf24rx_toDmx512_task", 2048, NULL, 1, &DMXWire::xNrf24rx_to_dmx512_taskhandler, NRF24_CORE);
      break;
      default:break;
   }
}

/*
stops every task, that uses NRF24 communication
*/
void DMXWire::nrf24_stop(){
   if( xNrf24tx_taskhandler != NULL ){   //stop nrf24tx_task
      vTaskDelete( xNrf24tx_taskhandler );
      radio->closeReadingPipe(1); // set network address
      digitalWrite(config.ledTxpin, LOW);
      Serial.println("delete xSlave_dmx512rx_task");
      delay(500);
   }
   if( xNrf24rx_taskhandler != NULL ){   //stop nrf24rx_task
      vTaskDelete( xNrf24rx_taskhandler );
      radio->stopListening();
      radio->closeReadingPipe(1);
      Serial.println("delete xNrf24rx_task");
      delay(500);
   }
   if( xNrf24rx_to_dmx512_taskhandler != NULL ){   //stop nrf24rx_toDmx512_task
      vTaskDelete( xNrf24rx_to_dmx512_taskhandler );
      radio->stopListening();
      radio->closeReadingPipe(1);
      Serial.println("delete xNrf24rx_toDmx512_task");
      delay(500);
   }
   if( xDmx512_to_nrf24_taskhandler != NULL ){   //stop nrf24rx_toDmx512_task
      vTaskDelete( xDmx512_to_nrf24_taskhandler );
      radio->closeReadingPipe(1); // set network address
      Serial.println("delete dmx512_to_nrf24_task");
      delay(500);
   }
}