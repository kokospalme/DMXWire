/*	LC1 DMXWire example
 *
 *	Example: Slave RX + NRF24
 *
 * Testing timings:
 *  - 16...18ms für alle packets bei Clock=400000 (1 Frame mit 512 Bytes + 1 byte head)
 */
#include "Arduino.h"
#include <DMXWire.h>
#include <dmx.h>
#include <SPI.h>
#include <nRF24L01.h> // library: https://github.com/maniacbug/RF24
#include <RF24.h>

#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 5
#define SDA_PIN 4

#define LED0_PIN 8	//Luatosboard: 12 & 13, indicate wire
#define LED1_PIN 0	//indicate RX/TX 

#define LC1DMX_MODE_OFF 0
#define LC1DMX_MODE_TX_DMX512 1
#define LC1DMX_MODE_TX_NRF24 2
#define LC1DMX_MODE_RX_DMX512 3
#define LC1DMX_MODE_RX_NRF24 4

#define LC1DMX_LED_OFF 0
#define LC1DMX_LED_RX_DMX512 1
#define LC1DMX_LED_RX_NRF24 2
#define LC1DMX_LED_TX_DMX512 3
#define LC1DMX_LED_TX_NRF24 4

#define LC1DMX_TX_FOLLOW 0	//follow = send frame when Master is sending one
#define LC1DMX_TX_MIN_MS 20	//20ms = 50 FPS
#define LC1DMX_TX_DEFAULT_MS 30	//30ms = 33,3 FPS
#define LC1DMX_TX_MAX_MS 40	//40ms = 25 FPS

uint8_t testValue = 0;

struct lc1dmx_settings_t{
	uint8_t ioMode = LC1DMX_MODE_TX_NRF24;
	uint8_t txFramerate_ms = LC1DMX_TX_FOLLOW;
	uint8_t led0Mode = DMXWIRE_LED_RX;	//indicate RX from Wire
	uint8_t led1Mode = LC1DMX_LED_RX_DMX512;
	unsigned long wiretimeout_ms = 500;	//timeouts
	unsigned long dmx512timeout_ms = 500;
	unsigned long nrf24timeout_ms = 500;
}config;


//NRF24 stuff
#define NRF24_MAX_DMX_CHANNELS 512 // full DMX
#define NRF24_BYTES_PER_PACKET 16 // usable bytes per packet
#define NRF24_PACKET_OVERHEAD 2 // group and time stamp
#define NRF24_MAXPAYLOAD (NRF24_BYTES_PER_PACKET + NRF24_PACKET_OVERHEAD) // max payload size for nrf24l01
#define NRF24_MAXGROUPS (NRF24_MAX_DMX_CHANNELS / NRF24_BYTES_PER_PACKET) // 32 groups of 16 channels = 512 DMX channels

#define NRF24_CE_PIN 3 // Chip enable
#define NRF24_CSN_PIN 10 // Chip select not
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

unsigned long refreshTimer = 0;
unsigned long flashTimer = 0;
uint64_t RXTXaddress = 0xF0F0F0F0F0LL;
uint8_t RXTXchannel = 0;
uint8_t payload[NRF24_MAXPAYLOAD], shadow_DMX[NRF24_MAX_DMX_CHANNELS];
uint8_t timeStamp; 
bool group_send;

void nrf24InitTX(); //initialize nrf24
void nrf24TX();   //transmit over NRF24

void setup(){
	Serial.begin(115200);
	Serial.println("*** LC1DMX (TX NRF24 mode) testing ***");

	switch(config.ioMode){	//input/output mode
		case LC1DMX_MODE_OFF:	//mode off [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_OFF);
		break;

		case LC1DMX_MODE_TX_DMX512:	//mode tx dmx512 [Wire slave, DMX TX]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		DMX::Initialize(output);
		break;

		case LC1DMX_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
      Serial.println("TX NRF24 Mode");
      pinMode(LED1_PIN, OUTPUT);
      nrf24InitTX();   //init NRF24
      
		break;

		default:	//default [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;
	}

	if(config.led0Mode >= DMXWIRE_LED_OFF && config.led0Mode <= DMXWIRE_LED_TX)Dmxwire.setLed0(LED0_PIN, config.led0Mode);	//set LED0 (Wire data indicator)
	
}

void loop(){
	switch(config.ioMode){	//input/output mode
		case LC1DMX_MODE_OFF:	//mode off [Wire slave, no output]
		// Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!	
		// if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_OFF);
		break;

		case LC1DMX_MODE_TX_DMX512:	//mode tx dmx512 [Wire slave, DMX TX]
		while(true){
			if(config.led1Mode == LC1DMX_LED_RX_DMX512)digitalWrite(LED1_PIN, HIGH);

			for(int i = 1; i < 20; i++){	//ToDo: hier vllt pointer?
				DMX::Write(i,Dmxwire.read(i));
			}

			// DMX::Write(1,Dmxwire.read(i));

			// Serial.print(Dmxwire.getDuration());
			// Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", Dmxwire.read(1), Dmxwire.read(2), Dmxwire.read(3), Dmxwire.read(4), Dmxwire.read(5));

			if(config.led1Mode == LC1DMX_LED_RX_DMX512)digitalWrite(LED1_PIN, LOW);
			Dmxwire.getTimeout();	//check if slave gets data 
		}
		break;

		case LC1DMX_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
		// Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
      nrf24TX();  //transmit over NRF24
		break;

		default:	//default [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;
	}
	
}

void nrf24InitTX(){
   radio.begin();
   radio.setAutoAck(false);
   radio.setPayloadSize(NRF24_MAXPAYLOAD);
   radio.setPALevel(RF24_PA_HIGH);    
   radio.setDataRate(RF24_250KBPS); 
   radio.setRetries(0,0);
   radio.setChannel(RXTXchannel); // set the channel
   radio.openWritingPipe(RXTXaddress); // set network address
   radio.stopListening(); // start talking !
}

void nrf24TX() { //transmit via NRF24

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
         radio.write( payload, sizeof(payload) ); // dump payload to radio
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

void nrf24RX(){
   if ( radio.available() ) {

      radio.read( payload, sizeof(payload) ); // get data packet from radio 
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