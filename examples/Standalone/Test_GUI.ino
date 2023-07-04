/*
Example to test the GUI-board (PCA9557 for button input + SSD1306 OLED display)
*/

#include <Arduino.h>
#include <Wire.h>
#include <PCA9557.h>

#define I2C_CLOCK 400000

#define SCL_PIN 5
#define SDA_PIN 4

#define LED0_PIN 8	//Luatosboard: 12 & 13, indicate wire
#define LED1_PIN 0	//indicate RX/TX 

PCA9557 io(0x1E, &Wire); // 0x19 for iFarm4G board

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK);
  io.pinMode(0, INPUT); // Config IO0 of PCA9557 to INPUT mode
  io.pinMode(1, INPUT); // Config IO1 of PCA9557 to INPUT mode
  io.pinMode(2, INPUT); // Config IO2 of PCA9557 to INPUT mode
}

void loop() {
   int pin0 = 0;
   int pin1 = 0;
   int pin2 = 0;

   if(io.digitalRead(0) == true) pin0 = 1;
   if(io.digitalRead(1) == true) pin1 = 1;
   if(io.digitalRead(2) == true) pin2 = 1;

   Serial.printf("%i.%i.%i\n", pin0, pin1, pin2);
   delay(50);

}
