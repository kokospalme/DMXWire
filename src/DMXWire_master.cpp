#include <Arduino.h>
#include "DMXWire.h"

/*
(Master) Begins Master device's peripherals:
Wire.begin(), xSemaphoreCreateMutex(), slaveAddress to this
@param uint8_t scl: scl pin 
@param uint8_t sda: sda pin 
@param uint8_t slaveaddress: slave's address
@param uint32_t clock: frequency on I2C bus (default: 400000)
*/
void DMXWire::beginMaster(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock = 400000){
   sync_dmx = xSemaphoreCreateMutex(); //create semaphore
   sync_config = xSemaphoreCreateMutex(); //create semaphore
   Wire.setTimeOut(100);
	Wire.begin(sda, scl,clock);
	DMXWire::slaveAddress = slaveaddress;
}


/*
(Master) starts RX task for requesting DMX data from slave
@param none: requests whole universe from slave
*beginMaster() has to be executed before!
*/
void DMXWire::startMaster_rx(){  //get whole universe
   request.getWholeUniverse = true;
   // xTaskCreatePinnedToCore(DMXWire::masterRx_task, "master_rx_task", 1024, NULL, 1, NULL, NRF24_CORE);
}


/*
(Master) starts RX task for requesting DMX data from slave

*beginMaster() has to be executed before!:
@param uint16_t startChannel: start channel to request from slave
@param uint16_t noChannels: number of channels to request from slave
*/
void DMXWire::startMaster_rx(uint16_t startChannel, uint16_t noChannels){  //only get some channels
   request.getWholeUniverse = false;
   request.requestChannel = startChannel;
   request.requestNoChannels = noChannels;
   Serial.println("starting masterRx_task");
   xTaskCreatePinnedToCore(DMXWire::masterRx_task, "master_rx_task", 2048, NULL, 1, NULL, NRF24_CORE);
}


/*
(Master) Stops RX task for requestin DMX data from Slave
*/
void DMXWire::stopMaster_rx(){
   // deleteTask
}


/*
(Master) Callback function to send dmx data to slave every x ms.
function is called from main function with timer object

used config variables:
ledTxMode
*/
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


/*
(Master) Task for continous requesting DMX data from slave every x ms.

used config variables:
timeout_wire_ms
rxFramerate_ms
*/
void DMXWire::masterRx_task(void*pvParameters){
   dmxwire_request_t _request = Dmxwire.request;
   uint16_t i = 0;
   bool waiting = false;
   xSemaphoreTake(sync_config, portMAX_DELAY);
   unsigned long _timeout = config.timeout_wire_ms;
   uint16_t _framerate_ms = config.rxFramerate_ms;
   int _slaveAddress = 8;
   int _ledTxpin = config.ledTxpin;
   int _ledTxMode = config.ledTxMode;
   xSemaphoreGive(sync_config);

   for(;;){
      
      if(_request.getWholeUniverse){   //ToDo: implement


      }else{   //get dmx channel byte by byte


         if( millis() > _request.timer + _framerate_ms){
            _request.timer = millis();
            if(_ledTxMode == DMXWIRE_LED_WIRE) digitalWrite(_ledTxpin, HIGH);

            Dmxwire.requestDmx(_request.requestChannel + i);
            uint8_t bytesReceived = Wire.requestFrom(_slaveAddress, 3);  //wait for a 3 bytes message
            if((bool) bytesReceived){
               uint8_t temp[bytesReceived];
               Wire.readBytes(temp,bytesReceived); //receive bytes
               uint16_t _rxCh;
               uint16_t _reqCh = ((uint16_t)(temp[0]) << 8) | (uint16_t)temp[1]; //array to uin16_t
               if(temp[0] == _request.requestChannel + i){ //if correct channel received: write to Buffer
               xSemaphoreTake(sync_dmx, portMAX_DELAY);
               Dmxwire.write(_request.requestChannel + i, temp[1]);
               xSemaphoreGive(sync_config);
               }else{
               Serial.printf("received wrong channel.msg:%u.%u.%u\n", temp[0], temp[1], temp[2]);
               }
            }else{
               Serial.println("wire timed out");         
            }
            i++;
            if(i > _request.requestNoChannels -1) i = 0; //reset i
         }
      
      }
      delay(20);

      
   }

}


/*

(Master) Sends a DMX-channel request to slave and updates timestamp_wire.
3 bytes in total:

@params uint16_t asdaw

[0] = DMXWIRE_PACKET_DMXREQUEST (ID for dmx channel request)
[1] = channel lowbyte
[2] = chanel highbyte
*/
void DMXWire::requestDmx(uint16_t channel){
   uint8_t _packet[3];
   _packet[0]  = DMXWIRE_PACKET_DMXREQUEST;  //header 0: dmx request
   _packet[1] = (uint8_t) (channel >> 8);
   _packet[2] = (uint8_t) channel;

   Serial.printf("request ch:%u\n", channel);
   Wire.beginTransmission(slaveAddress); // transmit to device #xy
	Wire.write(_packet, 3);
	Wire.endTransmission(true);    // stop transmitting
   timestamp_wire = millis();
}


/*
(Master) sends a DMX-Packet to slave.

variable packetNo has to be set before
*/
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