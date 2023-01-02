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

#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 5
#define SDA_PIN 4

#define LED0_PIN 12	//Luatosboard: 12 & 13, indicate wire
#define LED1_PIN 13	//indicate RX/TX 

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
	uint8_t ioMode = LC1DMX_MODE_TX_DMX512;
	uint8_t txFramerate_ms = LC1DMX_TX_FOLLOW;
	uint8_t led0Mode = DMXWIRE_LED_RX;	//indicate RX from Wire
	uint8_t led1Mode = LC1DMX_LED_RX_DMX512;
	unsigned long wiretimeout_ms = 500;	//timeouts
	unsigned long dmx512timeout_ms = 500;
	unsigned long nrf24timeout_ms = 500;
}config;

void setup(){
	Serial.begin(115200);
	Serial.println("*** LC1DMX (TX DMX512 mode) testing ***");

	// Wire.begin(DMXWIRE_SLAVEADDRESS, SDA_PIN, SCL_PIN, I2C_CLOCK);
	// Wire.onReceive(slaveRXcallback);

	switch(config.ioMode){	//input/output mode
		case LC1DMX_MODE_OFF:	//mode off [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!	
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_OFF);
		break;

		case LC1DMX_MODE_TX_DMX512:	//mode tx dmx512 [Wire slave, DMX TX]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		DMX::Initialize(output);
		break;

		case LC1DMX_MODE_TX_NRF24:	//mode tx nrf24 [Wire slave, NRF24 TX]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;

		default:	//default [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;
	}

	if(config.led0Mode >= DMXWIRE_LED_OFF && config.led0Mode <= DMXWIRE_LED_TX)Dmxwire.setLed0(LED0_PIN, config.led0Mode);	//set LED0 (Wire data indicator)
	
	
}

void loop(){
	switch(config.ioMode){	//input/output mode
		case LC1DMX_MODE_OFF:	//mode off [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!	
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_OFF);
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
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;

		default:	//default [Wire slave, no output]
		Dmxwire.beginSlaveRX(SDA_PIN, SCL_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);	//ToDo: umdrehen später!
		if(config.led0Mode == DMXWIRE_LED_AUTO)Dmxwire.setLed0(LED0_PIN, DMXWIRE_LED_RX);
		break;
	}
	
}
