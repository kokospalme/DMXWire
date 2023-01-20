#include "DMXWire.h"

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
         Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_DMX512);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("DMX board TX DMX512 Mode");
         DMX::Initialize(output);
         break;

         case DMXBOARD_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
         Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_NRF24);
         setLedRx(DMXWIRE_LED_WIRE);
         Serial.println("DMX board TX NRF24 Mode");
         nrf24InitTX();   //init NRF24
         break;

         default:	//default [Wire slave, no output]
         Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
         setLedTx(DMXWIRE_LED_OFF);
         setLedRx(DMXWIRE_LED_WIRE);
         break;
	}
}


void DMXWire::nrf24InitTX(){
   SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, NRF24_CSN_PIN);
   
   radio->begin();
   radio->setAutoAck(false);
   radio->setPayloadSize(NRF24_MAXPAYLOAD);
   radio->setPALevel(RF24_PA_HIGH);    
   radio->setDataRate(RF24_250KBPS); 
   radio->setRetries(0,0);
   radio->setChannel(RXTXchannel); // set the channel
   radio->openWritingPipe(RXTXaddress); // set network address
   radio->stopListening(); // start talking !
}

void DMXWire::nrf24TX() { //transmit via NRF24

   for (uint8_t group = 0; group < NRF24_MAXGROUPS; group++) { // send groups of DMX data, 16 bytes at a time
      uint16_t group_ptr = group * NRF24_BYTES_PER_PACKET; // create group pointer for array
      if (millis() - refreshTimer > 1000) { // allow ALL radio data (full DMX array) to be send once per second, regardless of changes
         refreshTimer = millis(); // reset refresh timer

         group_send = true; // force ALL send
         
         
      }else { 
         group_send = false; // preset flag to false, only set it if there has been a data change since last time
      } 


      for (uint8_t chan = 0; chan < NRF24_BYTES_PER_PACKET; chan++) {
         if ( Dmxwire.read(group_ptr+chan) != shadow_DMX[group_ptr+chan] ) { // compare test : current DMX against old DMX 
            shadow_DMX[group_ptr+chan] = Dmxwire.read(group_ptr+chan); // if changed, update shadow array of DMX data and payload
            group_send = true; // set flag so that THIS group packet gets sent
         } 
         payload[chan+2] = Dmxwire.read(group_ptr+chan); // ensure ALL up-to-date data gets through on this packet
      } 
      

      if (group_send) { // only send the data that has changed, any data change in a group will result in whole group send
         payload[0] = group; // set first byte to point to group number (groups of 16 bytes)
         payload[1] = timeStamp++; // second byte helps us monitor consistency of reception at receiver with a continuous frame counter
         radio->write( payload, sizeof(payload) ); // dump payload to radio
         pinMode(LED1_PIN, HIGH);
         Serial.println("update!");
         delay(100);
      } 
      pinMode(LED1_PIN, LOW);
      delay(50);
   } 
   
   // if ( (DMXSerial.noDataSince() < 2000) && (millis() > 1000) ) { // if DMX has been received within the last 2 seconds make LED flash
   //    if (!(millis() & 0b1100000000)) { 
   //       digitalWrite(LED_RED,1); // flash ON
   //    } 
   //    else { // flash OFF
   //       digitalWrite(LED_RED,0); 
   //    }  
   // }else { // no DMX data present, no flash just ON
   //    digitalWrite(LED_RED,1); 
   // }

} 

void DMXWire::nrf24RX(){
   if ( radio->available() ) {

      radio->read( payload, sizeof(payload) ); // get data packet from radio 
      for (uint8_t i = 0; i < NRF24_BYTES_PER_PACKET; i++) {
         // DMXWire.write((NRF24_BYTES_PER_PACKET * payload[0])+i, payload[i+2]); // parse radio data into dmx data array
         
      } 
      int i = 0;
      Serial.println(payload[i+2]);


      // if (millis() - flashTimer < 750) { 
      //    digitalWrite(LED_BLUE,0);
      // } 
      // else if (millis() - flashTimer > 749) { 
      //    digitalWrite(LED_BLUE,1);
      // } 
      // if (millis() - flashTimer > 1024) { 
      //    flashTimer = millis(); // reset timer after 1 second(ish)
      // } 
   } 
}