#include <Arduino.h>
#include "DMXWire.h"

void DMXWire::beginMaster(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
	Wire.begin(sda, scl,clock);
	DMXWire::slaveAddress = slaveaddress;
}

void DMXWire::startMaster_rx(){  //get whole universe
   request.getWholeUniverse = true;
   // xTaskCreatePinnedToCore(DMXWire::masterRx_task, "master_rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
}

void DMXWire::startMaster_rx(uint16_t startChannel, uint16_t noChannels){  //only get some channels
   request.getWholeUniverse = false;
   request.requestChannel = startChannel;
   request.requestNoChannels = noChannels;
   // xTaskCreatePinnedToCore(DMXWire::masterRx_task, "master_rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
}

void DMXWire::stopMaster_rx(){
   // deleteTask
}


void DMXWire::masterTXcallback(){
	if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, HIGH);

	for(int i = 0; i < DMXWIRE_PACKETS; i++){	//ToDo: später mehr
		packetNo = i;

      xSemaphoreTake(sync_dmx, portMAX_DELAY);  //task safety
		packets[i][0] = packetNo;	//head: info which packet is being send
      xSemaphoreGive(sync_dmx);

		Dmxwire.sendPacketToSlave();	//send packet
      
	}

	if(config.ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(config.ledTxpin, LOW);
}

void DMXWire::masterRx_task(void*pvParameters){
   dmxwire_request_t _request = Dmxwire.request;
   uint16_t i = 0;
   for(;;){

      if(_request.getWholeUniverse){   //ToDo: implement


      }else{   //get dmx channel byte by byte
         requestDmx(_request.requestChannel + i);
         i++;
         if(i > _request.requestNoChannels -1) i = 0; //reset i
      }
      delay(5);
   }
}

void DMXWire::master_dmx512rx_task(void*pvParameters){
   // const uint16_t dmxFromChannel 
   // const uint16_t numberOfChannel 
   
   // for(;;){


   // }
}

void DMXWire::requestDmx(uint16_t channel){
   uint8_t _packet[4];
   _packet[0]  = DMXWIRE_PACKET_DMXREQUEST;  //header 0: dmx request
   _packet[1]  = channel;  //header 0: packet No
   _packet[2]  = channel << 8;  //header 0: packet No
   
   Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(_packet, DMXWIRE_BYTES_PER_PACKET);
	Wire.endTransmission();    // stop transmitting
   timestamp_wire = millis();

   xSemaphoreTake(sync_config, portMAX_DELAY);
   unsigned long _timeout = config.timeout_wire_ms;
   xSemaphoreGive(sync_config);

   while(millis() < timestamp_wire + _timeout){ //wait for slave to answer (within time)
      uint8_t bytesReceived = Wire.requestFrom(slaveAddress, 2);  //wait for a 2 bytes message
      if((bool) bytesReceived){
         uint8_t temp[bytesReceived];
         Wire.readBytes(temp,bytesReceived);

         if(temp[0] == channel){ //if correct channel received: write to Buffer
            xSemaphoreTake(sync_dmx, portMAX_DELAY);
            Dmxwire.write(channel, temp[1]);
            xSemaphoreGive(sync_config);
         }else{
            Serial.printf("received wrong byte:%u\n", temp);
         }
      }
      delay(1);
   }
   Serial.println("wire timeout");
}

void DMXWire::setPacket(){	//master TX
	Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(packetNo);              // sends one byte
	Wire.endTransmission();    // stop transmitting
}

void DMXWire::sendPacketToSlave(){	//master TX
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