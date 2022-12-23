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
#include <ESP32TimerInterrupt.h>

#define TIMER0_INTERVAL_MS        1000
#define TIMER0_DURATION_MS        5000


class DMXWire {
public:
	DMXWire(uint8_t timerNo);
	void setClock(uint32_t frequency);	//set I2C clock
	void beginSender(uint8_t scl,uint8_t sda, uint8_t slaveaddress );
	void beginReceiver(uint8_t scl,uint8_t sda );
	void write(uint16_t channel, uint8_t byte);
	uint8_t read(uint16_t channel);

	virtual ~DMXWire();
private:
	uint8_t packets[32][16];
	uint8_t packetNo = 0;
	uint8_t slaveAddress = 1;	//slave's address

	void requestData();
	void setPacket();
	void sendPacket();

	/** IR stuff **/
	bool IRAM_ATTR IRhandlerTX(void * timerNo);
	ESP32Timer *ITtimer;


};

#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
