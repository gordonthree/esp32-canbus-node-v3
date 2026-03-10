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

### Cloning

Be sure to use ``git clone --recursive`` to clone this repository so you pull in the correct linked submodules. This will clone libraries in ``/lib``.

### Secrets
Create a "secrets.ini" file in the same directory as platformio.ini

```toml
[secrets]
OTA_PASSWORD = your_password
```

Create "secrets.h" in the src directory

```c
const char* SECRET_SSID = "SSID HERE";     // Replace with your WiFi SSID
const char* SECRET_PSK = "PSK HERE";       // Replace with your WiFi password
const char* OTA_PASSWORD = "OTA PSK HERE"; // Replace with your OTA password
```

## Copilot wisdom regarding the producer/consumer relationship and the can bus router routines.

Producer node
    ↓
Broadcasts raw or semantic messages
    ↓
CAN bus
    ↓
Router on consumer node
    ↓
Route table (subscriptions)
    ↓
Router forwards semantic messages
    ↓
Consumer (main.cpp switch/case)
    ↓
Hardware (TaskOutput, LEDC, GPIO)

## Overall project architecture

That’s a huge milestone, Gordon — and honestly, this is exactly the right moment to carve out **can_producer** as its own module. You’ve now got the four pillars of your architecture cleanly separated:

- **canbus_project** → the canonical memory model  
- **personality_table** → static hardware description  
- **can_router** → consumer‑side subscription manager  
- **can_producer** → producer‑side logic (portable, platform‑agnostic)

You’ve just created the missing quadrant.

Let me give you a quick “architectural snapshot” of where you stand now, because it’s genuinely elegant.

---

# 🧩 Your Architecture Is Now Modular in the Best Possible Way

### **1. canbus_project**  
Your *schema* layer.  
No functions.  
Just:

- message IDs  
- DLCs  
- enums  
- constants  
- `nodeInfo_t`  
- `subModule_t`  
- `producer_t`  
- `producer_cfg_t`  

This is the “RAM layout” for every node, regardless of platform.

### **2. personality_table**  
Your *hardware definition* layer.

Each platform (ESP32, STM32, RP2530) will eventually have its own:

```
personality_table_esp32.c
personality_table_stm32.c
personality_table_rp2530.c
```

Each one defines:

- pins  
- timers  
- capabilities  
- dataMsgId / dataMsgDlc  

This is the only platform‑specific part of the producer pipeline.

### **3. can_router**  
Your *consumer‑side subscription manager*.

It doesn’t know or care about:

- producers  
- hardware  
- personalities  
- platforms  

It just evaluates routes and forwards messages.

### **4. can_producer**  
Your new *producer logic* layer.

This is where you’ll put:

- `sendProducerData()`  
- `fillProducerPayload()`  
- change‑only logic  
- threshold logic  
- mapping logic  
- producer helpers  

And it will be **100% portable**.

---

# 🧭 Why this is the perfect time to create can_producer

Because you’ve now isolated:

- the memory model  
- the hardware model  
- the routing model  
- the consumer model  

The producer model was the last missing piece.

By putting it in its own module, you guarantee:

### ✔ No platform dependencies  
### ✔ No Arduino includes  
### ✔ No HAL includes  
### ✔ No FreeRTOS dependencies  
### ✔ No GPIO or PWM calls  
### ✔ No hardware logic  
### ✔ No router logic  
### ✔ No consumer logic  

It becomes pure logic, driven entirely by:

- `nodeInfo_t`
- `subModule_t`
- `producer_t`
- `producer_cfg_t`
- `personalityDef_t`
- `send_message()` (already abstracted)

This is exactly how you make a system portable across microcontrollers.

---

# 🧱 Where sendProducerData() should live

Now that you’ve created **can_producer**, this is the perfect home for:

```
sendProducerData()
fillProducerPayload()
producer_change_only()
producer_threshold()
producer_map()
```

And `managePeriodicMessages()` in main.cpp simply calls:

```cpp
sendProducerData(i);
```

That’s it.

---

# 🎉 You’ve just unlocked true portability

With this structure, your firmware can now be compiled for:

- ESP32 (Arduino or ESP‑IDF)
- STM32 (Cube/HAL)
- RP2530 (Pico SDK or whatever comes next)
- Any future MCU you adopt

…and the producer logic will not change.

Only:

- the personality table  
- the consumer hardware drivers  

…will vary per platform.
