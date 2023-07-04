/**
 * DMX Example: DMX to Wire(NRF)
 * UART2 RX: 16 // has to be UART2 default RX pin
 * UART1 TX: 14 // 13 doesn't work
 * 
 * - both DMX % DMX1 classes run on core 0
 * - default UART pins have to be used to work glitch-free
 * 
 * 
*/

#include <Arduino.h>
#include <DMXWire.h>

#define SPI_MISO_PIN 19 
#define SPI_MOSI_PIN 23
#define SPI_SCK_PIN 18
#define NRF24_CE_PIN 15
#define NRF24_CSN_PIN 4

dmxwire_settings_t dmxwireCfg;
dmxwire_hardware_t dmxwireHardware;

uint8_t dmxValues[512];

void setup(){
  Serial.begin(115200);

  dmxwireHardware.nrf_ce = NRF24_CE_PIN;  //set dmxwire hardware
  dmxwireHardware.nrf_cs = NRF24_CSN_PIN;

  dmxwireCfg.ioMode = DMXBOARD_MODE_TX_NRF24; //set dmxwire config
  dmxwireCfg.ledRxpin = GPIO_NUM_NC;
  dmxwireCfg.ledTxpin = GPIO_NUM_NC;
  dmxwireCfg.txFramerate_ms = 29; //30,3ms = 30303 uS

  Dmxwire.setHardware(dmxwireHardware);
  
  Serial.println("HELLO");
  DMX::Initialize(input);
  // SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Dmxwire.beginStandalone();
  Dmxwire.setConfig(dmxwireCfg);  //beginStandalone first!
  Dmxwire.switchIomode();

}

void loop(){
  DMX::ReadAll(dmxValues,1,512);
  for(int i = 1; i < 513; i++){
    Dmxwire.write(i,dmxValues[i-1]);
  }
  delay(10);
}