/*
 * DMXWire.h
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#ifndef DMXWIRE_H_
#define DMXWIRE_H_
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <Ticker.h>

#define DMXWIRE_INTERVAL_MS	30	// 20 ... 40ms = 50 ... 25 FPS

#define DMXWIRE_CHANNEL_PER_PACKET 16
#define DMXWIRE_HEAD 1 
#define DMXWIRE_BYTES_PER_PACKET DMXWIRE_CHANNEL_PER_PACKET + DMXWIRE_HEAD
#define DMXWIRE_PACKETS 512 / DMXWIRE_CHANNEL_PER_PACKET

#define DMXWIRE_NOTBUSY -1

#define DMXWIRE_LED_OFF 0
#define DMXWIRE_LED_RX 1
#define DMXWIRE_LED_TX 2
#define DMXWIRE_LED_AUTO 3

#define DMXWIRE_PACKETTIMEOUT_MS 500 //ms

struct dmxwire_settings_t{
	int led0pin = LED_BUILTIN;	//built in LED
	uint8_t led0Mode = DMXWIRE_LED_OFF;
	int led1pin = -1;	//off
	uint8_t led1Mode = DMXWIRE_LED_OFF;

	unsigned long packettimeout_ms = DMXWIRE_PACKETTIMEOUT_MS;
};

class DMXWire {
public:
	DMXWire();
	static void setClock(uint32_t frequency);	//set I2C clock
	static void setLed0(int pin, uint8_t mode);	//set pin and mode for led0
	static void setLed0(uint8_t mode);	//set mode for led0
	static void setLed1(int pin, uint8_t mode);	//set pin and mode for led1
	static void setLed1(uint8_t mode);	//set mode for led1
	static void beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock);
	static void beginSlaveRX(uint8_t scl,uint8_t sda, uint8_t slaveaddress, uint32_t clock);

	static void write(uint16_t channel, uint8_t value);
	static uint8_t read(uint16_t channel);
	static unsigned long getDuration();	//get last duration in ms
	static void setTimout(unsigned long timeout_ms);
	static bool getTimeout();

		/** IR stuff **/
	static Ticker IRtimer;	// siehe: https://github.com/espressif/arduino-esp32/issues/3465
	virtual ~DMXWire();

	static void masterTXcallback();
	static void slaveRXcallback(int bufSize);
private:
	static uint8_t packets[DMXWIRE_PACKETS][DMXWIRE_BYTES_PER_PACKET];
	static uint8_t packetNo;
	static uint8_t slaveAddress;	//slave's address

	static void requestData();
	static void setPacket();
	static void sendPacket();

	static int packetBusy;	//is packet busy? 
	static unsigned long duration;	//capture framerate
	static unsigned long timestamp;

	static dmxwire_settings_t config;

   //task safety
   static SemaphoreHandle_t sync_dmx;

}; extern DMXWire Dmxwire;


#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
