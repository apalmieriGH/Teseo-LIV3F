# Teseo-LIV3F

GPSProvider wrapper library for STMicroelectronics' X-NUCLEO-GNSS1A1 Expansion Board.

## Overview

This is the Github repo including drivers for ST’s Teseo-LIV3F Global Navigation Satellite System (GNSS) device and middleware for the NMEA protocol support. 
This firmware package implements the port of the [GPSProvider](https://github.com/apalmieriGH/GPSProvider/) to STMicroelectronics' X-NUCLEO-GNSS1A1 GNSS Expansion Board.

The key features of the package are:
* Complete software to build applications using Teseo-LIV3F GNSS device
* Middleware for the NMEA protocol support

Furthermore the package provides the following advanced features:
* Geofencing - allows the Teseo-LIV3F receiver to raise a NMEA message when the resolved GNSS position is close to or entering or exiting from a specific circle
* Odometer - provides information on the traveled distance using only the resolved GNSS position
* Data Logging - allows the Teseo-LIV3F receiver to save locally on the flash the resolved GNSS position to be retrieved on demand from the Host

## Hardware description

The X-NUCLEO-GNSS1A1 is a Global Navigation Satellite System Expansion Board usable with the STM32 Nucleo system and other Arduino compatible platforms. It is designed around the STMicroelectronics Teseo-LIV3F GNSS receiver IC working on multiple constellations (GPS/Galileo/Glonass/BeiDou/QZSS).

The Teseo-LIV3F module is designed for top performance in a minimal space. Within its 10x10mm compact size, Teseo-LIV3F offers superior accuracy thanks to the on board 26MHz Temperature Compensated Crystal Oscillator (TCXO) and a reduced Time To First Fix (TTFF) relying to its dedicated 32KHz Real Time Clock (RTC) oscillator.

The X-NUCLEO-GNSS1A1, hosting the Teseo-LIV3F, is compatible with Arduino UNO R3 connector layout and interfaces with the MCU via the UART channel.
To connect by serial port the GNSS receiver and the host the following parameters must be used:
* 8 data bits
* No parity
* 1 stop bit
* 9600 bauds

A GPS/GLONASS/Beidou antenna, distributed along with the X-NUCLEO-GNSS1A1 Expansion Board, must be connected to the antenna connector present on the Expansion Board.
For the X-NUCLEO-GNSS1A1 proper operations, the following jumper settings must be used: 
* J2 open
* J3 closed
* J4 closed
* J5 open
* J6 closed
* J7 closed
* J8 open
* J9 closed
* J10 open
* J11 closed
* J12 closed
* J13 closed
* J14 closed
* J15 closed

## Tested platforms

This firmware has been tested on STM32 NUCLEO-F401RE

## Example Application

To run GNSS example applications using X-NUCLEO-GNSS1A1 Expansion Board based on mbed OS and built with [mbed-cli](https://github.com/ARMmbed/mbed-cli/), please refer to [TeseoLocation](https://github.com/apalmieriGH/TeseoLocation/) page.