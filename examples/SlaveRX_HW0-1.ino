/*	LC1 DMXWire example
 *
 *	Example: Slave RX + NRF24
 *
 * Testing timings:
 *  - 16...18ms für alle packets bei Clock=400000 (1 Frame mit 512 Bytes + 1 byte head)
 */
#include "Arduino.h"
#include <DMXWire.h>

void serialhandler();   //handles Serial commands

void setup(){
	Serial.begin(115200);

   while(1){
      Serial.println("*** LC1DMX (unimode) testing ***");
      delay(1000);
   }
   // SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, NRF24_CSN_PIN);   
   // Dmxwire.dmxboardInit(); //init dmxboard

}

void loop(){

	// Dmxwire.dmxboardRun();  //run dmxboard
   // serialhandler();
}



void serialhandler(){   //
   if(Serial.available()){
      String _cmd = Serial.readStringUntil('\n');  //read line
      String _cmd0 = _cmd.substring(0, _cmd.indexOf("."));
      String _cmd1 = _cmd.substring(_cmd.indexOf("."), _cmd.length());
      Serial.printf("CMD0:%c, CMD1:%c\n",_cmd0.c_str(), _cmd1.c_str());

   }
}
