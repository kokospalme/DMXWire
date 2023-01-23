#include <Arduino.h>
#include "DMXWire.h"

void DMXWire::slaveRXcallback(int bufSize){  //callback for Wire slave
   timestamp_wire = millis();
	uint8_t _counter = 0;
	uint8_t buffer[bufSize];
   if(bufSize > DMXWIRE_BYTES_PER_PACKET) return;	//return if buffer too large
	
	for(int i=0; i < bufSize; i++){  //read every byte to buffer
      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		buffer[i] = Wire.read();
      if(config.ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledRxpin, HIGH);
      xSemaphoreGive(sync_dmx);
		// Serial.print(buffer[i], HEX);
	}
	// Serial.println("");

	packetNo = buffer[0];
	

	if(packetNo < DMXWIRE_PACKETS){	// if packet in packetrange: write buffer to packet

		if(buffer[0] == DMXWIRE_PACKETS-1){ //duration for a whole packet-set of 512 bytes
			duration_wire = millis() - timestamp_wire;
		}

      for(int i = 0; i < DMXWIRE_BYTES_PER_PACKET; i++){
         packets[packetNo][i] = buffer[i];
      }

      // if(packetNo== 0){
      // 	Serial.print(Dmxwire.getDuration());
      // 	Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", packets[0][1], packets[0][2], packets[0][3], packets[0][4], packets[0][5]);
      // }


	}else{	//else: setting codes or unknown

      if(packetNo == DMXWIRE_PACKET_DMXREQUEST){ //dmx request

         packetNo = buffer[1];

         xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
         packets[packetNo][0] = packetNo;	//head: info which packet is being send
         xSemaphoreGive(sync_dmx);

         Dmxwire.sendPacketToMaster();	//send packet

      }else if(packetNo == DMXWIRE_PACKET_SETTINGS){  //settings
         if(bufSize < 5) return; //packet too small

         uint16_t _cmd0 = buffer[1] << 8 | buffer[2]; //LBHB
         uint16_t _cmd1 = buffer[3] << 8 | buffer[4]; //LBHB
         settingshandler(_cmd0, _cmd1);
      }
	}

	packetNo = DMXWIRE_NOTBUSY;	//reset "busy-variable"

}

void DMXWire::sendAck(uint16_t cmd){
   Wire.write(cmd);
}

void DMXWire::sendPacketToMaster(){	//Slave TX

   xSemaphoreTake(sync_dmx, portMAX_DELAY);  
	packetBusy = packetNo;
   uint8_t _packet[DMXWIRE_BYTES_PER_PACKET];
   for(int i = 0; i <DMXWIRE_BYTES_PER_PACKET; i++){
      _packet[i] = packets[packetNo][i];
   }
   xSemaphoreGive(sync_dmx);

	Wire.write(_packet, DMXWIRE_BYTES_PER_PACKET);
	packetBusy = DMXWIRE_NOTBUSY;
	// if(packetNo == 0)Serial.printf("P%u: %u \t%u \t%u \t%u \t%u\n", packets[packetNo][0], packets[packetNo][1], packets[packetNo][2], packets[packetNo][3], packets[packetNo][4], packets[packetNo][5]);
}

void DMXWire::settingshandler(uint16_t cmd0, uint16_t cmd1){
   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   dmxwire_settings_t _cfg = config;
   xSemaphoreGive(sync_config);

   if(cmd0 == SETTINGID_SAVE_TO_EEPROM){ //write to EEPROM
      Serial.println("write things to eeprom...");
      Dmxwire.writeConfig();
      sendAck(0);
   }else if(cmd0 == SETTINGID_RESTART_DEVICE){   //restart
      Serial.println("restart device");
      sendAck(0);
      ESP.restart();
   }else if(cmd0 == SETTINGID_HARDRESET){   //hardreset device
      Serial.println("hardreset Device...");
      dmxwire_settings_t _newConfig;
      xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
      config = _newConfig;
      xSemaphoreGive(sync_config);
      Dmxwire.writeConfig();
      sendAck((uint16_t)_cfg.ioMode);
      Serial.println("restart device");
      ESP.restart();
   }else if(cmd0 == SETTINGID_SET_IOMODE_IDLE){   //ioMode idle
      _cfg.ioMode = DMXBOARD_MODE_OFF;
      Serial.printf("set ioMode: idle(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_TX_DMX512){ //ioMode TX dmx512
      _cfg.ioMode = DMXBOARD_MODE_TX_DMX512;
      Serial.printf("set ioMode: RX dmx512(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_TX_NRF24){ //ioMode TX NRF24
      _cfg.ioMode = DMXBOARD_MODE_TX_NRF24;
      Serial.printf("set ioMode: tx nrf24(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_TX_NRF24){ //ioMode RX dmx512
      _cfg.ioMode = DMXBOARD_MODE_RX_DMX512;
      Serial.printf("set ioMode: tx dmx512(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_RX_NRF24){ //ioMode RX NRF24
      _cfg.ioMode = DMXBOARD_MODE_RX_NRF24;
      Serial.printf("set ioMode: rx nrf24(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_DMX512TONRF24){ //ioMode DMX to NRF24
      _cfg.ioMode = DMXBOARD_MODE_DMX512TONRF24;
      Serial.printf("set ioMode: dmx512 to nrf24(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_SET_IOMODE_NRF24TODMX512){ //ioMode DMX to NRF24
      _cfg.ioMode = DMXBOARD_MODE_NRF24TODMX512;
      Serial.printf("set ioMode: nrf24 to dmx512(%u)\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);
   }else if(cmd0 == SETTINGID_GET_IOMODE){ //get io Mode
      Serial.printf("get ioMode:%u\n", _cfg.ioMode);
      sendAck((uint16_t)_cfg.ioMode);


   }else if(cmd0 == SETTINGID_GET_DMX512_TIMEOUT){ //get dmx512 timeout
      Serial.printf("get dmx512 timeout:%u\n", _cfg.timeout_dmx512_ms);
      sendAck((uint16_t)_cfg.timeout_dmx512_ms);

   }else if(cmd0 == SETTINGID_SET_DMX512_TIMEOUT){ //set dmx512 timeout
      _cfg.timeout_dmx512_ms = (unsigned long) cmd1;
      Serial.printf("set dmx512 timeout:%u\n", _cfg.timeout_dmx512_ms);
      sendAck((uint16_t)_cfg.timeout_dmx512_ms);

   }else if(cmd0 == SETTINGID_GET_DMX512_FPS){ //get dmx512 transmit FPS
      Serial.printf("get dmx512 timeout:%u\n", _cfg.txFramerate_ms);
      sendAck((uint16_t)_cfg.txFramerate_ms);

   }else if(cmd0 == SETTINGID_SET_DMX512_FPS){ //set dmx512 transmit FPS
      _cfg.txFramerate_ms = (uint8_t) cmd1;
      Serial.printf("set dmx512 timeout:%u\n", _cfg.txFramerate_ms);
      sendAck((uint16_t)_cfg.txFramerate_ms);


   }else if(cmd0 == SETTINGID_GET_NRF24_TIMEOUT){ //get nrf24 timeout
      Serial.printf("get nrf24 timeout:%u\n", _cfg.timeout_nrf24_ms);
      sendAck((uint16_t)_cfg.timeout_nrf24_ms);

   }else if(cmd0 == SETTINGID_SET_NRF24_TIMEOUT){ //get nrf24 timeout
      _cfg.timeout_nrf24_ms = (unsigned long) cmd1;
      Serial.printf("set nrf24 timeout:%u\n", _cfg.timeout_nrf24_ms);
      sendAck((uint16_t)_cfg.timeout_nrf24_ms);

   }else if(cmd0 == SETTINGID_GET_NRF24_NOISE){ //get nrf24 timeout
      uint8_t _noise = random(0,255);  //ToDo: implement getNoise
      Serial.printf("get nrf24 noise:%u\n", _noise);
      sendAck((uint16_t)_noise);
   }else if(cmd0 == SETTINGID_SET_NRF24_CHANNEL){ //get nrf24 timeout
      if(cmd1 == SETTINGID_SET_NRF24_CHANNEL_AUTOMODE){
         _cfg.nrf_RXTXchannel = 0;
         Serial.println("set nrf24 channel: automode");
      }else{
         _cfg.nrf_RXTXchannel = (uint8_t) cmd1;
         Serial.printf("set nrf24 channel:%u\n", _cfg.nrf_RXTXchannel);
      }
      sendAck((uint16_t)_cfg.nrf_RXTXchannel);
   }else if(cmd0 == SETTINGID_GET_NRF24_CHANNEL){ //get nrf24 timeout
      Serial.printf("get nrf24 channel:%u\n", _cfg.nrf_RXTXchannel);
      sendAck((uint16_t)_cfg.nrf_RXTXchannel);
   }

   xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
   config = _cfg;
   xSemaphoreGive(sync_config);
   delay(2);

}

void DMXWire::dmxboardInit(){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   Serial.println("read config from preferences:");
   radio = new RF24(NRF24_CE_PIN, NRF24_CSN_PIN);  //init radio
   Dmxwire.preferencesInit();
   Dmxwire.readConfig();

   config.nrf_RXTXaddress = 0xF0F0F0F0F0LL;
   config.ledRxpin = LEDRX_PIN;  //overwrite pins
   config.ledTxpin = LEDTX_PIN;


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
            xTaskCreatePinnedToCore(DMXWire::nrf24tx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, NRF24_CORE);
         break;

         case DMXBOARD_MODE_RX_DMX512:	//mode rx dmx512 [Wire slave, DMX RX]
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_WIRE);
            setLedRx(DMXWIRE_LED_DMX512);
            Serial.println("DMX board RX DMX512 Mode");
            DMX::Initialize(input);
         break;

         case DMXBOARD_MODE_RX_NRF24:	//mode rx dmx512 [Wire slave, DMX RX]
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
            setLedTx(DMXWIRE_LED_WIRE);
            setLedRx(DMXWIRE_LED_NRF24);
            Serial.println("DMX board RX NRF24 Mode");
         break;

         case DMXBOARD_MODE_DMX512TONRF24:   //ToDo: implement
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_NRF24);
            setLedRx(DMXWIRE_LED_DMX512);
            DMX::Initialize(input);
            Serial.println("init DMX512TONRF24");
         break;

         case DMXBOARD_MODE_NRF24TODMX512:
            beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
            setLedTx(DMXWIRE_LED_DMX512);
            setLedRx(DMXWIRE_LED_NRF24);
            DMX::Initialize(output);
            xTaskCreatePinnedToCore(DMXWire::nrf24rx_toDmx512, "nrf24_rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
            Serial.println("init NRF24TODMX512");
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
         Serial.printf("TX_DMX512. ch1:%u\n",DMX::Read(1));  //read from Dmx buffer
         delay(20);

      break;
      case DMXBOARD_MODE_TX_NRF24:
         Serial.printf("TX_NRF24. ch1:%u\n",nrf24.shadow_DMX[1]);  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_RX_DMX512:

         Serial.println("MODE_RX_DMX512");
         delay(500);
      break;

      case DMXBOARD_MODE_RX_NRF24:
         Serial.printf("RX_NRF24. ch1:%u\n",Dmxwire.read(1));  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_DMX512TONRF24:
         Serial.printf("DMX512TONRF24. ch1:%u\n",nrf24.shadow_DMX[1]);  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_NRF24TODMX512:
         Serial.printf("NRF24TODMX512. ch1:%u\n",Dmxwire.read(1));  //read from nrf24 buffer
         delay(100);
      break;

      default:
      break;
   }
   Dmxwire.serialhandlerSlave();
}