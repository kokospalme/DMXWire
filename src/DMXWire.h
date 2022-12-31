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
//#include <ESP32TimerInterrupt.h>

#define TIMER_INTERVAL_MS        35
#define DMXW_PACKETSIZE 32
#define DMXW_PACKETS 512 / DMXW_PACKETSIZE

class DMXWire {
public:
	DMXWire(uint8_t timerNo);
	void setClock(uint32_t frequency);	//set I2C clock
	void beginSender(uint8_t scl,uint8_t sda, uint8_t slaveaddress );
	void beginReceiver(uint8_t scl,uint8_t sda );
	void runSender();
	void runReceiver();
	void write(uint16_t channel, uint8_t byte);
	uint8_t read(uint16_t channel);

	virtual ~DMXWire();
private:
	unsigned long timer = 0;
	uint8_t buffer[DMXW_PACKETS][DMXW_PACKETSIZE];
	uint8_t packetNo = 0;
	uint8_t slaveAddress = 1;	//slave's address

	void requestData();
	void setPacket();
	void sendPacket();

//	/** IR stuff **/
//	bool IRAM_ATTR IRhandlerTX(void * timerNo);
//	ESP32Timer *ITtimer;


};

#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
