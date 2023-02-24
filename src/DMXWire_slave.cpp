#include <Arduino.h>
#include "DMXWire.h"


/*
(Slave) callback function when Master sent Data.
timestamp_wire is begin updated.
*/
void DMXWire::slaveRXcallback(int bufSize){  //callback for Wire slave
   timestamp_wire = millis(); //update timestamp_wire
	uint8_t _counter = 0;
	uint8_t buffer[bufSize];
   if(bufSize > DMXWIRE_BYTES_PER_PACKET) return;	//return if buffer too large
      xSemaphoreTake(sync_config, portMAX_DELAY);  //task safety
      uint8_t _ledRxMode = config.ledRxMode;
      int _ledRxPin = config.ledTxpin;
      xSemaphoreGive(sync_config);
	
	for(int i=0; i < bufSize; i++){  //*read every byte to buffer
      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		buffer[i] = Wire.read();
      if(_ledRxMode == DMXWIRE_LED_WIRE) digitalWrite(_ledRxPin, HIGH);
      xSemaphoreGive(sync_dmx);
		// Serial.print(buffer[i], DEC);
      // Serial.print(".");
	}
	// Serial.println("");

	packetNo = buffer[0];
	

	if(packetNo < DMXWIRE_PACKETS){//* case: Master is sending a DMX packet
   // Serial.println("HIER_A");
		if(buffer[0] == DMXWIRE_PACKETS-1){ //duration for a whole packet-set of 512 bytes
			duration_wire = millis() - timestamp_wire;
		}

      for(int i = 0; i < DMXWIRE_BYTES_PER_PACKET; i++){
         packets[packetNo][i] = buffer[i];
      }

      
      #ifdef   DMXWIRE_DEBUG_SLAVE_WIRE
      // Serial.print("got DMX from master:");
      // if(packetNo== 0){
      //    Serial.print(Dmxwire.getDuration());
      //    Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", packets[0][1], packets[0][2], packets[0][3], packets[0][4], packets[0][5]);
      // }
      #endif



	}else if(buffer[0] == DMXWIRE_PACKET_DMXREQUEST){ //* case: Master is requesting a single DMX packet
   // Serial.printf("RX:%u.%u.%u\n", buffer[0], buffer[1], buffer[2]);
   // Serial.println("HIER_B");
         uint8_t txBuffer[3] = {0,0,0};
         uint16_t _ch = (uint8_t) buffer[1] << 8 | (uint8_t) buffer[2];

         txBuffer[0] = (uint8_t) (_ch >> 8);
         txBuffer[1] = (uint8_t) _ch;


         
         if(_ch > 0 && _ch <=512){
            txBuffer[2] = Dmxwire.read(_ch);
            #ifdef   DMXWIRE_DEBUG_SLAVE_WIRE
            Serial.printf("Master requests DMX(ch:%u) tx value:%u\n", _ch, txBuffer[2]);
            #endif
         }else{
            #ifdef   DMXWIRE_DEBUG_SLAVE_WIRE
            Serial.printf("Master requests DMX(ch:%u) E:out of range\n", _ch);
            #endif
         }
         // Serial.printf("TX:%u.%u.%u\n", txBuffer[0], txBuffer[1], txBuffer[2]);
         // Serial.printf("send to Master(ch:%u)%u.%u.%u\n", _ch, txBuffer[0], txBuffer[1], txBuffer[2]);
         Wire.slaveWrite(txBuffer, 3);

         

      }else if(buffer[0] == DMXWIRE_PACKET_DMXREQUEST_PACKET){ //* case: Master is requesting a DMX packet
      // Serial.println("HIER_C");

         if(buffer[1] >= DMXWIRE_PACKETS){
            #ifdef   DMXWIRE_DEBUG_SLAVE_WIRE
            Serial.printf("Master requests DMX packet(%u) E:unknown\n", buffer[1]);
            #endif
            return;
         }
         #ifdef   DMXWIRE_DEBUG_SLAVE_WIRE
         Serial.printf("Master requests DMX packet(%u)\n", buffer[1]);
         #endif

         packetNo = buffer[1];
         xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
         packets[packetNo][0] = packetNo;	//head: info which packet is being send
         xSemaphoreGive(sync_dmx);
         Dmxwire.sendPacketToMaster();	//send packet
      }else if(buffer[0] == DMXWIRE_PACKET_SETTINGS){  //settings
         if(bufSize < 5) return; //packet too small
         uint16_t _cmd0 = buffer[1] << 8 | buffer[2]; //LBHB
         uint16_t _cmd1 = buffer[3] << 8 | buffer[4]; //LBHB
         settingshandler(_cmd0, _cmd1);
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
   }else if(cmd0 == SETTINGID_SET_IOMODE_RX_DMX512){ //ioMode RX dmx512
      _cfg.ioMode = DMXBOARD_MODE_RX_DMX512;
      Serial.printf("set ioMode: rx dmx512(%u)\n", _cfg.ioMode);
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
         beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         Serial.println("init MODE_TX_NRF24");
         DMX::Initialize(input);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_WIRE);
         xTaskCreatePinnedToCore(DMXWire::nrf24tx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, NRF24_CORE);
      break;

      case DMXBOARD_MODE_RX_DMX512:	//mode rx dmx512 [Wire slave, DMX RX]
         beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init MODE_RX_DMX512");
         DMX::Initialize(input);
         xTaskCreatePinnedToCore(DMXWire::slave_dmx512rx_task, "nrf24_tx_task", 1024, NULL, 1, NULL, DMX512_CORE);
      break;

      case DMXBOARD_MODE_RX_NRF24:	//mode rx dmx512 [Wire slave, DMX RX]
         beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_WIRE);
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init MODE_RX_NRF24");
         xTaskCreatePinnedToCore(DMXWire::nrf24rx_task, "nrf24rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
      break;

      case DMXBOARD_MODE_DMX512TONRF24:   //ToDo: implement
         beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_DMX512);
         Serial.println("init DMX512TONRF24");
         DMX::Initialize(input);
         xTaskCreatePinnedToCore(DMXWire::dmx512_to_nrf24_task, "dmx512_to_nrf24_task", 1024, NULL, 1, &DMXWire::xDmx512_to_nrf24_taskhandler, DMX512_CORE);
      break;

      case DMXBOARD_MODE_NRF24TODMX512:
         beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_DMX512);
         setLedRx(DMXWIRE_LED_NRF24);
         Serial.println("init NRF24TODMX512");
         DMX::Initialize(output);
         xTaskCreatePinnedToCore(DMXWire::nrf24rx_toDmx512_task, "nrf24rx_toDmx512_task", 2048, NULL, 1, &DMXWire::xNrf24rx_to_dmx512_taskhandler, NRF24_CORE);
      break;

      default:	//default [Wire slave, no output]
         setLedTx(DMXWIRE_LED_OFF);
         setLedRx(DMXWIRE_LED_WIRE);
      break;
	}
}

void DMXWire::dmxboardRun(){
   bool _healthy = false;

   switch(config.ioMode){
      case DMXBOARD_MODE_OFF:
      break;
      case DMXBOARD_MODE_TX_DMX512:
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, HIGH);
         for(int i = 1; i <= 512; i++){   //write all 521 bytes from Wire to DMX Serial output

            DMX::Write(i, Dmxwire.read(i));
         }
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
         // Serial.printf("TX_DMX512. ch1:%u\n",DMX::Read(1));  //read from Dmx buffer
         delay(20);

      break;
      case DMXBOARD_MODE_TX_NRF24:
         // Serial.printf("TX_NRF24. ch1:%u\n",nrf24.shadow_DMX[1]);  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_RX_DMX512:
         
         xSemaphoreTake(sync_dmx, portMAX_DELAY);
         if(slave_status.dmx512_healthy == true)_healthy = true;
         else _healthy = false;
         xSemaphoreGive(sync_dmx);
         
         if(_healthy){
            digitalWrite(config.ledRxpin, HIGH);
            // Serial.printf("MODE_RX_DMX512. %u.%u.%u.%u.%u\n", Dmxwire.read(1),Dmxwire.read(2),Dmxwire.read(3),Dmxwire.read(4),Dmxwire.read(5));
         }else{
            digitalWrite(config.ledRxpin, LOW);
            // Serial.print("MODE_RX_DMX512. FAIL\n");
         }

         delay(100);
      break;

      case DMXBOARD_MODE_RX_NRF24:
         // Serial.printf("RX_NRF24. ch1:%u\n",Dmxwire.read(1));  //read from nrf24 buffer
         delay(100);
      break;
      case DMXBOARD_MODE_DMX512TONRF24:
         
         xSemaphoreTake(sync_dmx, portMAX_DELAY);
         if(slave_status.dmx512_healthy == true)_healthy = true;
         else _healthy = false;

         slave_status.lastDmxPacket = DMX::getLastPacket();
         xSemaphoreGive(sync_dmx);
         
         if(_healthy){
            digitalWrite(config.ledRxpin, HIGH);
            // Serial.printf("MODE_RX_DMX512. %u.%u.%u.%u.%u\n", Dmxwire.read(1),Dmxwire.read(2),Dmxwire.read(3),Dmxwire.read(4),Dmxwire.read(5));
         }else{
            digitalWrite(config.ledRxpin, LOW);
            // Serial.print("MODE_RX_DMX512. FAIL\n");
         }

         delay(100);
      break;
      case DMXBOARD_MODE_NRF24TODMX512:
         // Serial.printf("NRF24TODMX512. ch1:%u\n",Dmxwire.read(1));  //read from nrf24 buffer
         delay(100);
      break;

      default:
      break;
   }
   Dmxwire.serialhandlerSlave();
}