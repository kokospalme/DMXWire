/*
 * DMXWire.h
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 */

#ifndef DMXWIRE_H_
#define DMXWIRE_H_
#include "dmxboard.h"  //dedicated device
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <Ticker.h>

//dmxboard stuff
#include <dmx.h>
#include <SPI.h>
#include <nRF24L01.h> // library: https://github.com/maniacbug/RF24
#include <RF24.h>

#define DMXWIRE_INTERVAL_MS	30	// 20 ... 40ms = 50 ... 25 FPS

#define DMXWIRE_CHANNEL_PER_PACKET 16
#define DMXWIRE_HEAD 1 
#define DMXWIRE_BYTES_PER_PACKET DMXWIRE_CHANNEL_PER_PACKET + DMXWIRE_HEAD
#define DMXWIRE_PACKETS 512 / DMXWIRE_CHANNEL_PER_PACKET

#define DMXWIRE_NOTBUSY -1

#define DMXWIRE_LED_OFF 0  //LED modes
#define DMXWIRE_LED_DMX512 1
#define DMXWIRE_LED_NRF24 2
#define DMXWIRE_LED_WIRE 3

#define DMXWIRE_PACKETTIMEOUT_MS 500 //ms

#define SETTINGSID_NONE -1
#define SETTINGID_SAVE_TO_EEPROM 0
#define SETTINGID_RESTART DEVICE 1
#define SETTINGID_HARDRESET 2
#define SETTINGID_SET_SLAVE_ADD 10
#define SETTINGID_SET_SLAVEMODE 11
#define SETTINGID_DMX512_GET_TIMEOUT 20
#define SETTINGID_DMX512_SET_FPS 21
#define SETTINGID_NRF24_GET_TIMEOUT 30
#define SETTINGID_NRF24_GET_NOISE 31
#define SETTINGID_NRF24_SET_CHANNEL 32
#define SETTINGID_NRF24_GET_CHANNEL 33


struct dmxwire_settings_t{
   uint8_t ioMode = DMXBOARD_MODE_TX_DMX512; //default iomode: TX over DMX512 (Serial)
	int ledRxpin = LED_BUILTIN;	//built in LED
	int ledTxpin = -1;	//off
	uint8_t txFramerate_ms = DMXBOARD_TX_FOLLOW; //transmit only when master is sending something
	uint8_t ledRxMode = DMXWIRE_LED_WIRE;	//indicate RX (default: Wire)
	uint8_t ledTxMode = DMXWIRE_LED_DMX512;  //indicate TX (default: DMX512)
	unsigned long timeout_wire_ms = 500;	//timeouts
	unsigned long timeout_dmx512_ms = 500;
	unsigned long timeout_nrf24_ms = 500;
};

class DMXWire {
public:
	DMXWire();
   virtual ~DMXWire();
	static void setClock(uint32_t frequency);	//set I2C clock
	static void setLedRx(int pin, uint8_t mode);	//set pin and mode for led0
	static void setLedRx(uint8_t mode);	//set mode for led0
	static void setLedTx(int pin, uint8_t mode);	//set pin and mode for led1
	static void setLedTx(uint8_t mode);	//set mode for led1
	static void beginMasterTX(uint8_t scl, uint8_t sda, uint8_t slaveaddress, uint32_t clock);
	static void beginSlaveRX(uint8_t scl, uint8_t sda, uint8_t slaveaddress, uint32_t clock);

	static void write(uint16_t channel, uint8_t value);
	static uint8_t read(uint16_t channel);
	static unsigned long getDuration();	//get last duration in ms
   static void setTimout_wire(unsigned long timeout_ms);
   static void setTimout_dmx512(unsigned long timeout_ms);
	static void setTimout_nrf24(unsigned long timeout_ms);
	static bool getTimeout_wire();
   static bool getTimeout_dmx512();
   static bool getTimeout_nrf24();

   /* dedicate devices*/
   static void dmxboardInit();   //initializes dmx board (current hardware: v0.1)
   static void dmxboardRun();

   /** read/write settings **/
   static int writeSetting(uint8_t iD, int value); //writes a setting to slave
   static int writeSetting_saveTOEEPROM();
   static void writeSetting_restartSlave();
   static void writeSetting_hardresetSlave();
   static void writeSetting_setSlaveAddress(uint8_t address);
   static void writeSetting_setSlavemode(int value);
   static bool readSetting_timeoutDmx512();
   static bool readSetting_timeoutNRF24();
   static uint8_t readSetting_NRF24noise(uint8_t channel);
   static void writeSetting_NRF24channel(int channel);   // 0...255, -1: automatic
   static int readSetting_NRF24channel();


   /** IR stuff **/
	static Ticker IRtimer;	// siehe: https://github.com/espressif/arduino-esp32/issues/3465
	static void masterTXcallback();
	static void slaveRXcallback(int bufSize);
private:
	static uint8_t packets[DMXWIRE_PACKETS][DMXWIRE_BYTES_PER_PACKET];
	static uint8_t packetNo;
	static uint8_t slaveAddress;	//slave's address

	static void requestData();
	static void setPacket();
	static void sendPacket();

   /* dedicated devices */
   static RF24 *radio;
   static nrf24Data_t nrf24;  //data for nrf24
   static void nrf24InitTX(); //initialize nrf24 module (TX mode)
   static void nrf24InitRX(); //initialize nrf24 module (RX mode)
   static void nrf24TX();   //transmit over nrf24 module
   static void nrf24RX();
   

	static int packetBusy;	//is packet busy? 
	static unsigned long duration;	//capture framerate
	static unsigned long timestamp_wire;
   static unsigned long timestamp_dmx512;
   static unsigned long timestamp_nrf24;

	static dmxwire_settings_t config;

   //task safety
   static SemaphoreHandle_t sync_dmx;

}; extern DMXWire Dmxwire;


#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
