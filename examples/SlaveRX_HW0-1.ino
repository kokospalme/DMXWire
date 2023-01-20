/*	LC1 DMXWire example
 *
 *	Example: Slave RX on real Hardware v0.1
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
	Serial.println("*** DMXWire Slave RX starting ***");

	DMX::Initialize(output);

	Dmxwire.beginSlaveRX(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
	if(config.led0Mode >= DMXWIRE_LED_OFF && config.led0Mode <= DMXWIRE_LED_TX)Dmxwire.setLed0(LED0_PIN, config.led0Mode);	//set LED0 (Wire data indicator)
}

void loop(){
	
	for(int i = 1; i <= 512; i++){
		DMX::Write(i, Dmxwire.read(i));
	}
	
	Serial.print(Dmxwire.getDuration());
	Serial.printf("\t%u \t%u \t%u \t%u \t%u \n", Dmxwire.read(1), Dmxwire.read(2), Dmxwire.read(3), Dmxwire.read(4), Dmxwire.read(5));
	delay(LC1DMX_TX_DEFAULT_MS);

	Dmxwire.getTimeout();	//check if slave gets data 
}



