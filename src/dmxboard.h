#include <Arduino.h>
#include "DMXWire.h"

/* LED stuff*/
#define LEDRX_PIN 8	//LEDs red
#define LEDTX_PIN 0	//green 


#define DMXBOARD_TX_FOLLOW 0	//follow = send frame when Master is sending one
#define DMXBOARD_TX_MIN_MS 20	//20ms = 50 FPS
#define DMXBOARD_TX_DEFAULT_MS 30	//30ms = 33,3 FPS
#define DMXBOARD_TX_MAX_MS 40	//40ms = 25 FPS

#define DMXBOARD_MODE_OFF 0
#define DMXBOARD_MODE_TX_DMX512 1
#define DMXBOARD_MODE_TX_NRF24 2
#define DMXBOARD_MODE_RX_DMX512 3
#define DMXBOARD_MODE_RX_NRF24 4
#define DMXBOARD_MODE_DMX512TONRF24 5
#define DMXBOARD_MODE_NRF24TODMX512 6

/* WIRE STUFF */
#define I2C_CLOCK 400000
#define DMXWIRE_SLAVEADDRESS 8
#define SCL_PIN 5
#define SDA_PIN 4

/* NRF24 stuff*/
#define NRF24_MAX_DMX_CHANNELS 512 // full DMX
#define NRF24_BYTES_PER_PACKET 16 // usable bytes per packet
#define NRF24_PACKET_OVERHEAD 2 // group and time stamp
#define NRF24_MAXPAYLOAD (NRF24_BYTES_PER_PACKET + NRF24_PACKET_OVERHEAD) // max payload size for nrf24l01
#define NRF24_MAXGROUPS (NRF24_MAX_DMX_CHANNELS / NRF24_BYTES_PER_PACKET) // 32 groups of 16 channels = 512 DMX channels

#define SPI_CLK_PIN 5   //SPI settings
#define SPI_MISO_PIN 2
#define SPI_MOSI_PIN 7
#define NRF24_CE_PIN 3 // Chip enable
#define NRF24_CSN_PIN 10 // Chip select not


struct nrf24Data_t{
   unsigned long refreshTimer = 0;
   unsigned long flashTimer = 0;
   uint64_t RXTXaddress = 0xF0F0F0F0F0LL;
   uint8_t RXTXchannel = 0;   //rx/tx channel (0 ... 255)
   uint8_t payload[NRF24_MAXPAYLOAD];
   uint8_t shadow_DMX[NRF24_MAX_DMX_CHANNELS];  //buffer dmx
   uint8_t timeStamp; 
   bool group_send;
};



