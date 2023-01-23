/*	DMXWire library
 *
 *	Example: Master RX simple
 * --> request some DMX Data from slave
 *
 */
#include "Arduino.h"
#include <DMXWire.h>

#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 16
#define SDA_PIN 22

#define DMXCHANNEL 1 // requested channel: 1
#define NOCHANNELS 5
uint8_t ioMode = DMXBOARD_MODE_TX_DMX512;

void serialHandler();   //handles cmds from Serial

void setup(){
	Serial.begin(115200);
   delay(2000);
   Serial.println("DMXWire Master RX demo Booting...");

   Dmxwire.beginMaster(SCL_PIN, SDA_PIN, DMXWIRE_SLAVEADDRESS, I2C_CLOCK);
   Dmxwire.startMaster_rx(DMXCHANNEL, NOCHANNELS);  //request channel 1 ... 5 from DMXWire slave
   Serial.println("hier bin ich");
   uint8_t bytesReceived = Wire.requestFrom(DMXWIRE_SLAVEADDRESS, 2);  //wait for a 2 bytes message
}

void loop(){
   serialHandler();
}

void serialHandler(){
   String cmd = "";
   if(Serial.available()){
      cmd = Serial.readStringUntil('\n'); //read whole line
      cmd.trim();
      Serial.printf("cmd:%s\n",cmd.c_str());
      if(cmd.equals("help") == true){
         Serial.println("==== DMXWire commandline ====");
         Serial.println("R\tstart reading mode");
         Serial.println("R0\tstart requesting whole universe from slave");
         Serial.println("R1.5\tstart requesting particular DMX channels from slave");
         Serial.println("W\tstart writing mode");
         Serial.println("W10.22");
         Serial.println("getSettings\treads settings from slave and prints them");
      }else if(cmd.equals("R")){
         Serial.println("starting Reading mode");
      }

   }
}


