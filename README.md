# DMXWire
kokospalme/DMXWire
Arduino Library to send and receive DMX over I2C/Wire library

* uses interrupt handling from Ticker library (works)
* timing: 16...18ms for transmitting 512 bytes in 16+1 packets

## ToDos
* threadsafety with mutex (like in DMX library: https://github.com/luksal/ESP32-DMX) for multicore cpus
* I2C transaction with locks (also threadsafety)
* works with other libraries?! (DMX512 & NRF24 from https://forum.arduino.cc/t/transmitting-receiving-dmx-using-nrf24l01-radio-transceivers/203255)


## used libraries ##
https://github.com/espressif/arduino-esp32/tree/master/libraries/Ticker
https://github.com/espressif/arduino-esp32/tree/master/libraries/Wire
https://github.com/khoih-prog/ESP32TimerInterrupt

