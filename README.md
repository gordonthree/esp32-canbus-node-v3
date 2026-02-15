# PlatformIO Project - ESP32 CAN-bus Human Interface Node

This project supports a few different configuration options and ESP32 board types. See the ``platformio.ini`` file for details

Right now the most time has been spent working with the ESP32-CYD board. The ESP32-PICO is also supported, for controlling addressable RGB LEDs.

## Hardware
- ESP32-CYD - esp32-wroom
- ESP32-PICO - m5stamp-pico
- ESP32-S3 - m5stack-stamps3
- ESP32?? - Waveshare 7in touch screen

## Goals
- [x] WiFi Working
- [x] Non blocking WiFi connect
- [x] Keypad demo - four buttons that send CAN bus messages to remote switch nodes
- [x] Color picker demo - choose from 32 colors that send CAN bus messages to remote ARGB nodes
- [x] Node browser demo - Display other modules that have introduced themselves on the CAN bus
- [x] WiFi OTA
- [ ] WiFi Manager / Captive portal setup
- [x] FreeRTOS tasks
- [x] Create include file for CAN message ids
- [x] Setup CAN transcievers between two nodes

## Important setup notes

Be sure to use ``git clone --recursive`` to clone this repository so you pull in the submodule containing all the canbus data files (https://github.com/gordonthree/can-canbus-data)

You can also manually clone that repo into the ``lib/`` directory

Create a "secrets.ini" file in the same directory as platformio.ini

```
[secrets]
OTA_PASSWORD = your_password
```

