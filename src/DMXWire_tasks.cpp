#include <Arduino.h>
#include "DMXWire.h"

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

         
         unsigned long _stamp = xTaskGetTickCount();

         if( _stamp >= DMX::getLastPacket() && _stamp <= DMX::getLastPacket() + DMX_MAX_RXTIME_TICKS){

            // Serial.print(_stamp);Serial.print(",");Serial.print(DMX::getLastPacket()); Serial.print(",");Serial.println(DMX::Read(1));
            for(int i = 1; i <= 512 ;i++){   //write from DMX to Wire buffer
               // Dmxwire.write(i, _buffer[i-1]);
               Dmxwire.write(i, DMX::Read(i));
            }

            // delay(5);
         }
         
      }else{
         if(_ledRxMode == DMXWIRE_LED_DMX512) digitalWrite(_ledRxPin, LOW);
      }


      delay(1);
   }
}

void DMXWire::nrf24tx_task(void*pvParameters) { //transmit via NRF24

   radio->openWritingPipe(config.nrf_RXTXaddress); // set network address

   unsigned long _timer = millis(); //timestamp

   xSemaphoreTake(sync_config, portMAX_DELAY);
   uint8_t _framerate  = config.txFramerate_ms;
   xSemaphoreGive(sync_config);

   for(;;){

      if(millis() >= _timer + _framerate){
         unsigned long _time = millis() - _timer;
         Serial.printf("%u \n",_time);
         _timer = millis();
         

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
            if(config.ledTxMode == DMXWIRE_LED_NRF24 && config.ledTxpin >=0) digitalWrite(config.ledTxpin, HIGH);   //LED high
               timestamp_nrf24 = millis(); // reset timestamp
               nrf24.payload[0] = group; // set first byte to point to group number (groups of 16 bytes)
               nrf24.payload[1] = nrf24.timeStamp++; // second byte helps us monitor consistency of reception at receiver with a continuous frame counter
               // Serial.println("radio OK. send payload...");
               radio->write( nrf24.payload, sizeof(nrf24.payload) ); // dump payload to radio
            } 
         } 
         if(config.ledTxMode == DMXWIRE_LED_NRF24 && config.ledTxpin >=0) digitalWrite(config.ledTxpin, LOW);

         xSemaphoreTake(sync_config, portMAX_DELAY);
         uint8_t _iomode = config.ioMode;
         xSemaphoreGive(sync_config);
         if(_iomode != DMXBOARD_MODE_TX_NRF24){ //if ioMode is not longer TX NRF24, delete task
            if(config.ledTxpin >=0)digitalWrite(config.ledTxpin, LOW);
         }
         
      }

      delay(2);   //threadsafety
   }
} 

void DMXWire::nrf24rx_task(void*pvParameters){

   radio->openReadingPipe(1,config.nrf_RXTXaddress); // set network address
   radio->startListening(); // start listening for data

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
         digitalWrite(config.ledRxpin, LOW);
      }
      if(millis() > timestamp_nrf24 + _timestamp){ //LED handling
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
      }

   }
}

void DMXWire::nrf24rx_toDmx512_task(void*pvParameters){

   radio->openReadingPipe(1,config.nrf_RXTXaddress); // set network address
   radio->startListening(); // start listening for data

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
         delay(2);
         
      } 

      xSemaphoreTake(sync_config, portMAX_DELAY);
      uint8_t _iomode = config.ioMode;
      unsigned long _timestamp = config.timeout_nrf24_ms;
      xSemaphoreGive(sync_config);

      if(millis() > timestamp_nrf24 + _timestamp){ //LED handling
         if(config.ledRxMode == DMXWIRE_LED_NRF24) digitalWrite(config.ledRxpin, LOW);
         if(config.ledTxMode == DMXWIRE_LED_DMX512) digitalWrite(config.ledTxpin, LOW);
      }
   }
}

void DMXWire::dmx512_to_nrf24_task(void*pvParameters){
   uint8_t _iomode = 0;
   uint8_t _ledRxMode = 0;
   int _ledRxPin = 0;
   bool _healthy = 0;

   radio->openWritingPipe(config.nrf_RXTXaddress); // set network address

   for(;;){
      long _stamp = xTaskGetTickCount();
      xSemaphoreTake(sync_dmx, portMAX_DELAY);
      // long _lastDmx  = slave_status.lastDmxPacket;
      long _lastDmx = DMX::getLastPacket();
      xSemaphoreGive(sync_dmx);
      if(nrf24.radioOK == false) return;  //return if nrf24 is not initialized


      if( _stamp >= _lastDmx && _stamp <= _lastDmx + DMX_MAX_TXTIME_TICKS){
         for(int i = 1; i <= 512 ;i++){   //write from DMX to Wire buffer
            // Dmxwire.write(i, _buffer[i-1]);
            Dmxwire.write(i, DMX::Read(i));
         }


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
               // Serial.println(Dmxwire.read(1));
               if(config.ledTxMode == DMXWIRE_LED_NRF24 && config.ledRxpin >= 0) digitalWrite(config.ledTxpin, HIGH);
               timestamp_nrf24 = millis(); // reset timestamp
               nrf24.payload[0] = group; // set first byte to point to group number (groups of 16 bytes)
               nrf24.payload[1] = nrf24.timeStamp++; // second byte helps us monitor consistency of reception at receiver with a continuous frame counter
               // Serial.println("radio OK. send payload...");
               radio->write( nrf24.payload, sizeof(nrf24.payload) ); // dump payload to radio
            } 
         } 
         if(config.ledTxMode == DMXWIRE_LED_NRF24) if(config.ledRxpin >= 0)digitalWrite(config.ledTxpin, LOW);
         Serial.printf("%u %u\n",_lastDmx ,Dmxwire.read(1));

         xSemaphoreTake(sync_config, portMAX_DELAY);
         uint8_t _iomode = config.ioMode;
         xSemaphoreGive(sync_config);
         if(_iomode != DMXBOARD_MODE_TX_NRF24){ //if ioMode is not longer TX NRF24, delete task
            if(config.ledRxpin >= 0) digitalWrite(config.ledTxpin, LOW);
         }
         
      }

      delay(2);   //threadsafety

   }
}