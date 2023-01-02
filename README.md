# DMXWire
#### kokospalme/DMXWire

Arduino Library to send and receive DMX over I2C/Wire library
Shield: [![CC BY 4.0][cc-by-shield]][cc-by]

This work is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png
[cc-by-shield]: https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg

* uses interrupt handling from Ticker library (works)
* timing: 16...18ms for transmitting 512 bytes in 16+1 packets

## ToDos
* threadsafety with mutex (like in DMX library: https://github.com/luksal/ESP32-DMX) for multicore cpus
* I2C transaction with locks (also threadsafety)
* works with other libraries?! (DMX512 & NRF24 from https://forum.arduino.cc/t/transmitting-receiving-dmx-using-nrf24l01-radio-transceivers/203255)

### Tests & examples
#### Code
* MasterTX_esp32.ino: sucessfully tested on ESP32 
* SlaveRX_esp32.ino: sucessfully tested on ESP32 C3
* SlaveRX_TX512_simple.ino: sucessfully tested on ESP32 C3 with full 512 channel
#### Schematic
* DMX schematic from http://www.mathertel.de/Arduino/DMXShield.aspx
<img src="http://www.mathertel.de/Arduino/DMXShield/DMXShieldSchema.png">

## used libraries
Many thanks to Khoih-prog and luksal!
- https://github.com/espressif/arduino-esp32/tree/master/libraries/Ticker
- https://github.com/espressif/arduino-esp32/tree/master/libraries/Wire
- https://github.com/khoih-prog/ESP32TimerInterrupt
- fork of https://github.com/luksal/ESP32-DMX

