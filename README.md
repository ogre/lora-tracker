# SUSF LoRa HAB Tracker System [![Build Status](https://travis-ci.org/philcrump/lora-tracker.svg?branch=master)](https://travis-ci.org/philcrump/lora-tracker)

Phil currently uses the following combination:

### *slim-board/* - Eagle layout of a slimline HAB Tracker PCB

* AAA Energizer Ultimate L92 Lithium power source
* ADP1607 DC-DC Boost Regulator
* STM32F030 Microcontroller
* u-blox MAX-M8x GNSS Receiver
* RFM98W 434MHz LoRa Transceiver Module

### *firmware-tracker-phil/* - Firmware for LoRa/RTTY HAB Trackers

* libopencm3-based firmware
* payload-switched config system
* u-blox GNSS aid data storage
