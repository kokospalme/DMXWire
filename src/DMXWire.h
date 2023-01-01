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

#define DMXWIRE_INTERVAL_MS	300	// 20 ... 40ms = 50 ... 25 FPS

#define DMXWIRE_BYTES_PER_PACKET 16
#define DMXWIRE_PACKETS 512 / DMXWIRE_BYTES_PER_PACKET

#define DMXWIRE_NOTBUSY -1

class DMXWire {
public:
	DMXWire();
	static void setClock(uint32_t frequency);	//set I2C clock
	static void beginMasterTX(uint8_t scl,uint8_t sda, uint8_t slaveaddress );
	// static void beginMasterRX(uint8_t scl,uint8_t sda );
	static void write(uint16_t channel, uint8_t value);
	static uint8_t read(uint16_t channel);

		/** IR stuff **/
	static Ticker IRtimer;	// siehe: https://github.com/espressif/arduino-esp32/issues/3465
	virtual ~DMXWire();

	static void masterTXcallback();
private:
	static uint8_t packets[DMXWIRE_BYTES_PER_PACKET][DMXWIRE_PACKETS];
	static uint8_t packetNo;
	static uint8_t slaveAddress;	//slave's address

	static void requestData();
	static void setPacket();
	static void sendPacket();

	static int packetBusy;

}; extern DMXWire Dmxwire;


#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
