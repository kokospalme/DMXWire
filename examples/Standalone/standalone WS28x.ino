/*
DMXWire example: WS2813

This example works with the following hardware:
 - DMX board RXTX v0.2
 wiring:
 LED DATA: "DMX DI" pad on v0.2
 LED GND: "DMX GNDA" pad on v0.2

 the board gets DMX data from XLR or NRF24 module and pushes the data to a String of WS2813.
 The Adafruit Neopixel Library is used due to flickering problems with FastLED.
 
*/

#include <Arduino.h>
#include <DMXWire.h>
#include <Adafruit_NeoPixel.h>

#define WS28X_PIN 4  //hardwarepin for WS28x
#define NUM_LEDS 20  //number of LEDs

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, WS28X_PIN, NEO_GRB + NEO_KHZ400);

struct configstruct_t{ //struct to store config in
   bool wireless = false;  //sets rx to nrf24(wireless) or XLR (wire)
   uint16_t dmxAddress = 1;   //dmxaddress
};
configstruct_t config;

void setup() {
  Serial.begin(115200);
  Dmxwire.beginStandalone();

  if(config.wireless){  //switch input mode
   Dmxwire.setIomode(DMXBOARD_MODE_RX_NRF24);
  }else{
   Dmxwire.setIomode(DMXBOARD_MODE_RX_DMX512);
  }

   strip.begin();
   strip.setBrightness(0);
   strip.show(); // Initialize all pixels to 'off'

}

void loop() {
   strip.setBrightness(Dmxwire.read(config.dmxAddress)); //read first channel (brightness)

   for(int i = 0; i < NUM_LEDS; i++){
      strip.setPixelColor(i, Dmxwire.read(config.dmxAddress+1), Dmxwire.read(config.dmxAddress+2), Dmxwire.read(config.dmxAddress+3));
   }
   strip.show();  //push Data to LEDs

}