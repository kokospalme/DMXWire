#include <Arduino.h>
#include "DMXWire.h"

void DMXWire::beginMaster(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   Wire.setTimeOut(100);
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
   Serial.println("starting masterRx_task");
   xTaskCreatePinnedToCore(DMXWire::masterRx_task, "master_rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
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
   bool waiting = false;
   xSemaphoreTake(sync_config, portMAX_DELAY);
   unsigned long _timeout = config.timeout_wire_ms;
   uint16_t _framerate_ms = config.rxFramerate_ms;
   int _slaveAddress = 8;
   xSemaphoreGive(sync_config);

   for(;;){
      
      if(_request.getWholeUniverse){   //ToDo: implement


      }else{   //get dmx channel byte by byte


         if( millis() > _request.timer + _framerate_ms){
            _request.timer = millis();

            Dmxwire.requestDmx(_request.requestChannel + i);
            uint8_t bytesReceived = Wire.requestFrom(_slaveAddress, 2);  //wait for a 2 bytes message
            if((bool) bytesReceived){
               uint8_t temp[bytesReceived];
               Wire.readBytes(temp,bytesReceived);

               if(temp[0] == _request.requestChannel + i){ //if correct channel received: write to Buffer
               xSemaphoreTake(sync_dmx, portMAX_DELAY);
               Dmxwire.write(_request.requestChannel + i, temp[1]);
               xSemaphoreGive(sync_config);
               }else{
               Serial.printf("received wrong byte:%u\n", temp);
               }
            }else{
               Serial.println("wire timed out");         
            }
            i++;
            if(i > _request.requestNoChannels -1) i = 0; //reset i
         }
      
      }
      delay(2);
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
   Serial.printf("request ch:%u\n", channel);
   Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(_packet, DMXWIRE_BYTES_PER_PACKET);
	Wire.endTransmission(true);    // stop transmitting
   timestamp_wire = millis();
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