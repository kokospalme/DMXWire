/*
 * DMXWire.h
 *
 *  Created on: 23 Dec 2022
 *      Author: Chris
 * 
 * 
 *  measure power consumption:
 *  
 *  idle:      150mA
 *  TX NRF24:  150mA
 *  RX NRF24:  190mA
 */

#ifndef DMXWIRE_H_
#define DMXWIRE_H_
#include "dmxboard.h"  //dedicated device
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <Ticker.h>
#include <Preferences.h>

//dmxboard stuff
#include <dmx.h>
#include <SPI.h>
#include <nRF24L01.h> // library: https://github.com/maniacbug/RF24
#include <RF24.h>

#define DMXWIRE_DEBUG_DMXBOARD_RUN
// #define DMXWIRE_DEBUG_SLAVE_WIRE
// #define DMXWIRE_DEBUG_SLAVE_RX
// #define DMXWIRE_DEBUG_SLAVE_TX

#define DMX_MAX_RXTIME_TICKS 3
#define DMX_MAX_TXTIME_TICKS 3
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

#define DMXWIRE_PACKET_SETTINGS 255
#define DMXWIRE_PACKET_DMXREQUEST DMXWIRE_PACKETS
#define DMXWIRE_PACKET_DMXREQUEST_PACKET DMXWIRE_PACKETS + 1

#define SETTINGSID_NONE -1
#define SETTINGID_SAVE_TO_EEPROM 0  //saves settings to EEPROM
#define SETTINGID_RESTART_DEVICE 1  //restarts the slave device
#define SETTINGID_HARDRESET 2    //saves default settings to EEPROM  and reset slave device
#define SETTINGID_SET_IOMODE_TX_DMX512 10
#define SETTINGID_SET_IOMODE_TX_NRF24 11
#define SETTINGID_SET_IOMODE_RX_DMX512 12
#define SETTINGID_SET_IOMODE_RX_NRF24 13
#define SETTINGID_SET_IOMODE_DMX512TONRF24 14
#define SETTINGID_SET_IOMODE_NRF24TODMX512 15
#define SETTINGID_SET_IOMODE_IDLE 18
#define SETTINGID_GET_IOMODE 19

#define SETTINGID_GET_DMX512_TIMEOUT 20
#define SETTINGID_SET_DMX512_TIMEOUT 21
#define SETTINGID_GET_DMX512_FPS 22
#define SETTINGID_SET_DMX512_FPS 23
#define SETTINGID_GET_NRF24_TIMEOUT 30
#define SETTINGID_SET_NRF24_TIMEOUT 31
#define SETTINGID_GET_NRF24_NOISE 32
#define SETTINGID_SET_NRF24_CHANNEL 33
#define SETTINGID_SET_NRF24_CHANNEL_AUTOMODE 256
#define SETTINGID_GET_NRF24_CHANNEL 34

// multicore stuff
#define DMX512_CORE 0
#define NRF24_CORE 0

struct dmxwire_request_t{
   unsigned long timer = 0;
   uint16_t txCmd0 = 0;
   uint16_t txCmd1 = 0;
   uint16_t requestChannel = 0;
   uint16_t requestNoChannels = 0;
   bool getWholeUniverse = false;
};

struct dmxwire_settings_t{
   uint8_t ioMode = DMXBOARD_MODE_TX_DMX512; //default iomode: TX over DMX512 (Serial)
	int ledRxpin = LED_BUILTIN;	//built in LED
	int ledTxpin = -1;	//off
	uint16_t txFramerate_ms = DMXBOARD_TX_FOLLOW; //transmit only when master is sending something
   uint16_t rxFramerate_ms = 30; //request DMX every __ ms
	uint8_t ledRxMode = DMXWIRE_LED_WIRE;	//indicate RX (default: Wire)
	uint8_t ledTxMode = DMXWIRE_LED_DMX512;  //indicate TX (default: DMX512)
   uint64_t nrf_RXTXaddress = 0xF0F0F0F0F0LL;
   uint8_t nrf_RXTXchannel = 0;   //rx/tx channel (0 ... 255)
	unsigned long timeout_wire_ms = 100;	//timeouts
	unsigned long timeout_dmx512_ms = 500;
	unsigned long timeout_nrf24_ms = 500;
};

struct dmxwire_status_t{
   bool dmx512_healthy = false;
   bool nrf24_healthy = false;
   long lastDmxPacket = 0;
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
   static void setIomode(uint8_t mode);

   static void beginStandalone();   //initialize Library as standalone device (Wire is not initialized)
	static void beginMaster(uint8_t scl, uint8_t sda, uint8_t slaveaddress, uint32_t clock);
   static void startMaster_rx();
   static void startMaster_rx(uint16_t startChannel, uint16_t noChannels);
   static void stopMaster_rx();
	static void beginSlaveRX(uint8_t scl, uint8_t sda, uint8_t slaveaddress, uint32_t clock);
   static void switchIomode();   //starts 

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
   static void serialhandlerSlave();   //handles serial input for Slave
   static void serialhandlerMaster();  //handles serial input for Master
   static void settingshandler(uint16_t cmd0, uint16_t cmd1);
   static void requestDmx(uint16_t channel);   //request dmx from Slave
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

   static dmxwire_request_t request;
	static void requestData();
	static void setPacket();
	static void sendPacketToSlave();
   static void sendPacketToMaster();
   static void sendAck(uint16_t cmd);  //send to master over Wire
   static void masterRx_task(void*pvParameters);

   /* dedicated devices */
   static RF24 *radio;
   static bool rf24Initialized;
   static nrf24Data_t nrf24;  //data for nrf24

   //master's tasks
   static void master_dmx512rx_task(void*pvParameters);
   static TaskHandle_t xMaster_dmx512rx_taskhandler;

   //slave's tasks
   static void slave_dmx512rx_task(void*pvParameters);
   static void nrf24tx_task(void*pvParameters);   //transmit over nrf24 module
   static void nrf24rx_task(void*pvParameters);
   static void nrf24rx_toDmx512_task(void*pvParameters);
   static void dmx512_to_nrf24_task(void*pvParameters);
   
   static TaskHandle_t xSlave_dmx512rx_taskhandler;
   static TaskHandle_t xNrf24tx_taskhandler;
   static TaskHandle_t xNrf24rx_taskhandler;
   static TaskHandle_t xNrf24rx_to_dmx512_taskhandler;
   static TaskHandle_t xDmx512_to_nrf24_taskhandler;


   static dmxwire_status_t slave_status;
   

	static int packetBusy;	//is packet busy? 
	static unsigned long duration_wire;	//capture framerate
	static unsigned long timestamp_wire;
   static unsigned long timestamp_dmx512;
   static unsigned long timestamp_nrf24;

   // config stuff 
	static dmxwire_settings_t config;
   static Preferences *preferences;
   static void preferencesInit();
   static void readConfig();
   static void writeConfig();

   //task safety
   static SemaphoreHandle_t sync_dmx;
   static SemaphoreHandle_t sync_config;

}; extern DMXWire Dmxwire;


#endif /* LIBRARIES_DMXWIRE_DMXWIRE_H_ */
