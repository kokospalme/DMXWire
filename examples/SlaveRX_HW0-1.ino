/*	LC1 DMXboard example
 *
 *	Example: Slave RX + NRF24 (with dmxboard hardware)
 *
 * Testing timings:
 *  - 16...18ms für alle packets bei Clock=400000 (1 Frame mit 512 Bytes + 1 byte head)
 */
#include "Arduino.h"
#include <DMXWire.h>


void setup(){
	Serial.begin(115200);
   delay(3000);
   Serial.println("DMXWire slave RX (dmxboard) Booting...");
   SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);   

   Dmxwire.dmxboardInit(); //init dmxboard
}

void loop(){

	Dmxwire.dmxboardRun();  //run dmxboard

}

