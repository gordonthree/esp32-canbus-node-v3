/**
 * @file main.cpp
 * @brief Gateway controller for CAN-based ARGB and Vehicle Control system.
 * @details Handles Wi-Fi/OTA, TWAI (CAN) hardware lifecycle, and node discovery.
 * Acts as the bridge between the CYD UI and the distributed hardware nodes.
 * Additional roles: ARGB LED control
 * Planned roles: Interface with i2c and SPI sensors, LCD and OLED displays, and button / keypad input
 * 
 * @author Gordon McLellan
 * @date 2026-02-16
 */

#include <Arduino.h>
#include <ArduinoOTA.h>

#include <stdio.h>

#include <Preferences.h>
#include <rom/crc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* Load Wi-Fi networking */
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_wifi.h"

#ifdef ARGB_LED
/* Load FastLED */
#include <FastLED.h>
#endif

#ifdef ESP32CYD
#include "espcyd.h"
extern void registerARGBNode(uint32_t id); // bring function over from espcyd.cpp
#endif

/* Timekeeping library */
#include <time.h>

/* my secrets */
#include "secrets.h"

/* my colors */
#if defined(ARGB_LED) || defined(ESP32CYD) || defined(ARGBW_LED)
#include "colorpalette.h"
#endif

/**
 * @enum ConfigStatus
 * @brief Result codes for NVS operations.
 */
enum ConfigStatus {
    CFG_OK = 0,        /**< Load successful and CRC valid */
    CFG_ERR_MUTEX,     /**< Failed to acquire flash mutex */
    CFG_ERR_NOT_FOUND, /**< No configuration exists in NVS */
    CFG_ERR_CRC,       /**< Data found but CRC is invalid (corrupt) */
    CFG_ERR_NVS_OPEN   /**< Failed to open NVS namespace */
};


static SemaphoreHandle_t flashMutex = xSemaphoreCreateMutex(); /**< mutex for flash safety */

/* Task handles */
TaskHandle_t xTWAIHandle = NULL; /* declared and defined */

/* OTA task control */
volatile bool ota_enabled = false;
volatile bool ota_started = false;
const char* ota_password = SECRET_PSK; // change this

/* memory allocation for the flags */
uint8_t FLAG_SEND_INTRODUCTION = 0;
uint8_t FLAG_BEGIN_NORMAL_OPER = 0;
uint8_t FLAG_HALT_NORMAL_OPER  = 0;
uint8_t FLAG_SEND_HEALTHCHECK  = 0;
uint8_t FLAG_SEND_NODECHECK    = 0;
uint8_t FLAG_PRINT_TIMESTAMP   = 0;

/* my can bus stuff */
#include "canbus_project.h"
#include "byte_conversion.h"

#define CRC_INVALID_CONFIG     0xFFFF
#define MAX_SUB_MODULES        8
#define PWM_SCALING_FACTOR     (100U)

/* hardware definitions */
#define CYD_BACKLIGHT_PIN     21
#define CYD_LED_RED_PIN        4                
#define CYD_LED_BLUE_PIN      17
#define CYD_LED_GREEN_PIN     16
#define CYD_LDR_PIN           34
#define CYD_SPEAKER_PIN       26
#define M5STAMP_ARGB_PIN      27
#define M5STAMP_ARGB_COUNT     1
#define M5STAMP_BUTTON_PIN    39


/* dynamic discovery stuff */
nodeInfo_t node; /**< Store information about this node */
volatile uint16_t node_crc = 0xffff; /**< CRC-16 for the node configuration */
volatile int introMsgPtr;  /**< Pointer for the introduction and interview process */
volatile bool FLAG_VALID_CONFIG = false;

/* esp32 native TWAI CAN library */
#include "driver/twai.h"

/* Default CAN transceiver pins 
 * Set these as a build_flag in platformio.ini
*/
#ifndef RX_PIN
#define RX_PIN 22
#endif

#ifndef TX_PIN
#define TX_PIN 21
#endif

#ifndef NODEMSGID
#define NODEMSGID 0x701
#endif

/* CAN bus stuff: */
#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 100

volatile bool can_suspended = false;
volatile bool can_driver_installed = false;

unsigned long previousMillis = 0;  /* will store last time a message was sent */
String wifiIP;

static const char *TAG = "canesp32";

#define AP_SSID  "canesp32"

/* interrupt stuff */
hw_timer_t *Timer0_Cfg = NULL;
 
const char* ssid = SECRET_SSID;
const char* password = SECRET_PSK;
const char* hostname =AP_SSID;

volatile int i=4;
volatile bool isrFlag=false;
volatile bool ipaddFlag=true;

int period = 1000;
int8_t ipCnt = 0;

unsigned long time_now = 0;

#ifdef ARGB_LED
#define MAX_LEDS_PER_STRIP 255 /**< Maximum LEDs supported per submodule */
CRGB ledData[4][MAX_LEDS_PER_STRIP]; /**< Buffers for 4 possible strips / submodules */
#endif

unsigned long ota_progress_millis = 0;

volatile bool wifi_connected = false;
volatile uint8_t myNodeID[4]; /**< node ID comprised of four bytes from MAC address */ 

// void IRAM_ATTR Timer0_ISR()
// {
//   isrFlag = true;
// }

/**
 * @brief Handles specialized FastLED initialization for ARGB sub-modules.
 * @param index The sub-module index (used to index ledData arrays).
 * @param sub   Reference to the sub-module configuration.
 */
void initArgbHardware(uint8_t index, subModule_t& sub) {
#ifdef ARGB_LED
    uint8_t pin = sub.config.argbLed.outputPin;
    uint8_t count = sub.config.argbLed.ledCount;

    /* Max bounds check against pre-allocated memory */
    if (count > MAX_LEDS_PER_STRIP) {
        count = MAX_LEDS_PER_STRIP;
    }

    /** * FastLED requires static pin definitions. 
     * We map the dynamic config pin to the appropriate template. 
     */
    switch (pin) {
        case 18: FastLED.addLeds<WS2812B, 18, GRB>(ledData[index], count); break;
        case 19: FastLED.addLeds<WS2812B, 19, GRB>(ledData[index], count); break;
        case 25: FastLED.addLeds<WS2812B, 25, GRB>(ledData[index], count); break;
        case 26: FastLED.addLeds<WS2812B, 26, GRB>(ledData[index], count); break;
        case 27: FastLED.addLeds<WS2812B, 27, GRB>(ledData[index], count); break;
        default:
            Serial.printf("Error: Pin %d not hardware-capable for FastLED\n", pin);
            return;
    }
    Serial.printf("Submod %d: ARGB Init (Pin %d, Count %d)\n", index, pin, count);                
#endif
}

/**
 * @brief Initializes all sub-modules based on their intro message IDs.
 * @details Iterates through the sub-module array and calls the appropriate init function based on the intro message ID.
 * @note This function is called once by the main setup() function.
 */
void initHardware() {
    Serial.println("[HW] Initializing sub-modules...");

    for (int i = 0; i < MAX_SUB_MODULES; i++) {
        subModule_t& sub = node.subModule[i];
        
        if (sub.introMsgId == 0) continue;

        switch (sub.introMsgId) {
            case DISP_ARGB_LED_STRIP_ID:
                initArgbHardware(i, sub);
                break;

            case INPUT_DIGITAL_GPIO_ID:
                if (sub.config.digitalInput.outputRes == 1) { /* INPUT_RES_PULLUP */
                    pinMode(sub.config.digitalInput.inputPin, INPUT_PULLUP);
                } else if (sub.config.digitalInput.outputRes == 2) { /* INPUT_RES_PULLDOWN */
                    pinMode(sub.config.digitalInput.inputPin, INPUT_PULLDOWN);
                } else {
                    pinMode(sub.config.digitalInput.inputPin, INPUT);
                }
                Serial.printf("Submod %d: Digital Input Init (Pin %d)\n", i, sub.config.digitalInput.inputPin);
                break;

            case OUT_GPIO_DIGITAL_ID:
            case DISP_ANALOG_BACKLIGHT_ID:
            case DISP_ANALOG_LED_STRIP_ID:
            case DISP_STROBE_MODULE_ID:
                pinMode(sub.config.digitalOutput.outputPin, OUTPUT);
                /* Apply initial state if configured */
                if (sub.introMsgId == OUT_GPIO_DIGITAL_ID) {
                    digitalWrite(sub.config.digitalOutput.outputPin, sub.config.digitalOutput.outputMode ? HIGH : LOW);
                }
                Serial.printf("Submod %d: Output Init (Pin %d, Type 0x%03X)\n", i, sub.config.digitalOutput.outputPin, sub.introMsgId);
                break;

            case OUT_GPIO_PWM_ID:
                /* ESP32 LEDC setup: channel = i, frequency = config * 100, resolution = 8-bit */
                ledcSetup(i, (uint32_t)sub.config.pwmOutput.pwmFreq * PWM_SCALING_FACTOR, 8); 
                ledcAttachPin(sub.config.pwmOutput.outputPin, i);
                Serial.printf("Submod %d: PWM Output Init (Pin %d, Freq %d)\n", i, sub.config.pwmOutput.outputPin, (sub.config.pwmOutput.pwmFreq * PWM_SCALING_FACTOR));
                break;

            case INPUT_ANALOG_ADC_ID:
                pinMode(sub.config.analogInput.inputPin, ANALOG);
                Serial.printf("Submod %d: Analog Input Init (Pin %d)\n", i, sub.config.analogInput.inputPin);
                break;

            case DISP_TOUCHSCREEN_LCD_ID:
                Serial.printf("Submod %d: Touchscreen LCD Identified\n", i);
                break;

            default:
                Serial.printf("Submod %d: No hardware init defined for ID 0x%03X\n", i, sub.introMsgId);
                break;
        }
    }
}

/**
 * @brief Calculates a 16-bit CRC for the node configuration.
 * @details Uses the ESP32 ROM CRC16 implementation (CCITT).
 * @param node Reference to the canNodeInfo struct.
 * @return uint16_t The calculated checksum.
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value
    while (length--) {
        crc ^= (uint16_t)*data++ << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // Polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Calculates a 16-bit CRC for the entire node configuration.
 */
uint16_t getConfigurationCRC(const nodeInfo_t& node) {
    /* Hash the entire struct without worrying about internal fields */
    return crc16_ccitt((const uint8_t*)&node, sizeof(nodeInfo_t));
}

/**
 * @brief Erases the node configuration and CRC from NVS.
 * @return ConfigStatus status of the erase operation (OK, NOT_FOUND, or MUTEX).
 */
ConfigStatus eraseConfig() {
    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    if (!prefs.begin("node_cfg", false)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NVS_OPEN;
    }

    /* Check if the data key exists before attempting removal */
    if (!prefs.isKey("node_data")) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    prefs.remove("node_data");
    prefs.remove("node_crc");

    prefs.end();
    xSemaphoreGive(flashMutex);

    return CFG_OK;
}

/**
 * @brief Saves the node configuration to NVS and updates the external CRC.
 * @param node Reference to the nodeInfo_t struct to persist.
 * @return CFG_OK on success, or CFG_ERR_MUTEX if flash is busy.
 */
ConfigStatus saveConfig(const nodeInfo_t& node) {
    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    /* Open namespace in read/write mode */
    if (!prefs.begin("node_cfg", false)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NVS_OPEN; /* Return error if NVS open fails */
    }

    /** Calculate the CRC of the current RAM buffer before saving */
    uint16_t currentCrc = getConfigurationCRC(node);

    /** Write the configuration blob and the CRC key separately */
    prefs.putBytes("node_data", &node, sizeof(nodeInfo_t));
    prefs.putUShort("node_crc", currentCrc);

    prefs.end();
    xSemaphoreGive(flashMutex);

    return CFG_OK;
}


/**
 * @brief Loads the current node configuration and CRC from NVS.
 * @param[in,out] node Reference to the nodeInfo_t struct to be loaded.
 * @return ConfigStatus indicating the result of the operation.
 * @retval CFG_OK Load successful and CRC valid.
 * @retval CFG_ERR_MUTEX Failed to acquire flash mutex.
 * @retval CFG_ERR_NOT_FOUND No configuration exists in NVS.
 * @retval CFG_ERR_CRC Data found but CRC is invalid (corrupt).
 */
ConfigStatus loadConfig(nodeInfo_t& node) {
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    if (!prefs.begin("node_cfg", true)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }
    
    // Check if the key exists before reading
    if (!prefs.isKey("node_data")) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    // Read data and stored CRC
    prefs.getBytes("node_data", &node, sizeof(nodeInfo_t));
    uint16_t storedCrc = prefs.getUShort("node_crc", 0);
    
    prefs.end();
    xSemaphoreGive(flashMutex);

    // Validate
    if (getConfigurationCRC(node) != storedCrc) {
        return CFG_ERR_CRC;
    }

    return CFG_OK;
}

void TaskOTA(void *pvParameters) {
  /* Wait until WiFi is connected */
  while (!wifi_connected) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  /* Configure ArduinoOTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    can_suspended = true; /* Stop the task logic */

    /* stop timer0 isr if running */
    if (Timer0_Cfg != NULL) {
      timerAlarmDisable(Timer0_Cfg);
      timerDetachInterrupt(Timer0_Cfg);
    }

    /* 1. Kill the tasks using hardware first */
    if (xTWAIHandle != NULL) vTaskSuspend(xTWAIHandle);
#ifdef ESP32CYD    
    if (xDisplayHandle != NULL) vTaskSuspend(xDisplayHandle);
    if (xTouchHandle != NULL) vTaskSuspend(xTouchHandle);
#endif
    /* Stop the TWAI driver */
    twai_stop();
    // twai_driver_uninstall();
    
    Serial.println("OTA Start: App tasks suspended, starting flash... ");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
    ESP.restart(); /* Cleanest way to recover CAN hardware after a failed OTA */
  });

  /* Option A: Always enable OTA listener */
  ArduinoOTA.begin();
  ota_started = true;
  Serial.println("ArduinoOTA ready");

  /* Main loop: handle OTA requests */
  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  vTaskDelete(NULL);
}


/**
 * @brief Reads the ESP32 station MAC address and extracts a 4-byte Node ID.
 */
void readMacAddress() {
  uint8_t baseMac[6];
  esp_efuse_mac_get_default(baseMac); /* get the factory-burned MAC address */
  /* Copy 4 bytes starting from index 2 of baseMac into myNodeID */
  memcpy((void*)myNodeID, &baseMac[2], 4);

  Serial.print("Node ID extracted: ");
  for(int i = 0; i < 4; i++) {
    Serial.printf("%02X ", myNodeID[i]);
  }
  Serial.println();
}

/**
 * @brief Sets up the node info based on the node type
 * @details Configure information on the node and submodules based on the node type and data provided by the user
 * 
 * @param nodeType 
 */

void wifiOnConnect(){
  Serial.println("STA Connected");
  Serial.print("STA SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("STA IPv4: ");
  Serial.println(WiFi.localIP());
  wifiIP = WiFi.localIP().toString();
}

/* when wifi disconnects */
void wifiOnDisconnect(){
  Serial.println("STA disconnected, reconnecting...");
  delay(1000);
  WiFi.begin(ssid, password);
}

void WiFiEvent(WiFiEvent_t event){
    switch(event) {

        case SYSTEM_EVENT_AP_START:
            /* set ap hostname here */
            WiFi.softAPsetHostname(AP_SSID);
            /* enable ap ipv6 here */
            WiFi.softAPenableIpV6();
            break;

        case SYSTEM_EVENT_STA_START:
            /* set sta hostname here */
            WiFi.setHostname(AP_SSID);
            break;

        case SYSTEM_EVENT_STA_CONNECTED:
            /* enable sta ipv6 here */
            WiFi.enableIpV6();
            break;

        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            /* both interfaces get the same event */
            Serial.print("STA IPv6: ");
            Serial.println(WiFi.localIPv6());
            Serial.print("AP IPv6: ");
            Serial.println(WiFi.softAPIPv6());
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            wifiOnConnect();
            wifi_connected = true;
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            wifi_connected = false;
            wifiOnDisconnect();
            break;

        default:
            break;
    }
}

/**
 * @brief Send a message to the CAN bus
 * @param msgid The message ID of the frame to be sent
 * @param data The data to be sent in the frame
 * @param dlc The data length code of the frame, which is the number of bytes of data to be sent
 */
void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc) {
  twai_message_t message;
  static int failCount = 0; /* tx fail counter */

  /* Format message */
  message.identifier = msgid;       /**< set message ID */
  message.extd = 0;                 /**< 0 = standard frame, 1 = extended frame */
  message.rtr = 0;                  /**< 0 = data frame, 1 = remote frame */
  message.self = 0;                 /**< 0 = normal transmission, 1 = self reception request */
  message.dlc_non_comp = 0;         /**< non-compliant DLC (0-8 bytes) */
  message.data_length_code = dlc;   /**< data length code (0-8 bytes) */

  if (dlc > 8) dlc = 8; /* Safety check */
  memcpy(message.data, data, dlc);  /**< copy data to message data field */
  
  // Serial.printf("Sending message ID: 0x%03X\n", msgid);

  /* Attempt transmission with a 10ms timeout */
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
#ifdef ESP32CYD
    digitalWrite(LED_RED, LOW); /* Turn on RED LED */
#endif
    failCount = 0; /* Reset counter on successful queueing */
    Serial.printf("ID: 0x%03X queued\n", msgid);
  } else {
    failCount++;
    Serial.printf("Tx Fail (%d/3)\n", failCount);

    if (failCount >= 3) {
      Serial.println("Persistent failure: Initiating TWAI Recovery...");
      
      /* Physical Bus Recovery Sequence */
      twai_stop();
      twai_initiate_recovery(); 
      
      vTaskDelay(pdMS_TO_TICKS(100)); /* Short delay for hardware state change */
      
      twai_start();
      Serial.println("TWAI Restarted");
      
      failCount = 0; /* Reset after recovery attempt */
    }
  }
  // vTaskDelay(10);
#ifdef ESP32CYD
  digitalWrite(LED_RED, HIGH); /* Turn off RED LED */
#endif

}

static void setDisplayMode(uint8_t *data, uint8_t displayMode) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  uint16_t rxdisplayID = (data[4] << 8) | data[5]; // switch ID
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (displayMode) {
    case 0: // display off
      Serial.printf("Unit %d Display %d OFF\n", rxunitID, rxdisplayID);
      break;
    case 1: // display on
      Serial.printf("Unit %d Display %d ON\n", rxunitID, rxdisplayID);
      break;
    case 2: // clear display
      Serial.printf("Unit %d Display %d CLEAR\n", rxunitID, rxdisplayID);
      break;
    case 3: // flash display
      Serial.printf("Unit %d Display %d FLASH\n", rxunitID, rxdisplayID);
      break;
    default:
      Serial.println("Invalid display mode");
      break;
  }
}

static void setSwMomDur(uint8_t *data) {
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint16_t swDuration = (data[6] << 8) | data[7]; // duration in msD 
}


static void setSwBlinkDelay(uint8_t *data) {
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint16_t swBlinkDelay = (data[6] << 8) | data[7]; // delay in ms 
}

static void setSwStrobePat(uint8_t *data) {
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint8_t swStrobePat = data[6]; // strobe pattern
}


static void setPWMDuty(uint8_t *data) {
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint16_t PWMDuty = (data[6] << 8) | data[7]; // switch ID 
}

static void setPWMFreq(uint8_t *data) {
  uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint16_t PWMFreq = (data[6] << 8) | data[7]; // switch ID 
}
static void setSwitchMode(uint8_t *data) {
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  uint8_t switchMode = data[6]; // switch mode

  switch (switchMode) {
    case 0: // solid state (on/off)
      break;
    case 1: // one-shot momentary
      break;
    case 2: // blinking
      break;
    case 3: // strobing
      break;
    case 4: // pwm
      break;
    default:
      Serial.println("Invalid switch mode");
      break;
  }

}

static void txSwitchState(uint8_t *txUnitID, uint16_t txSwitchID, uint8_t swState) {
  uint8_t dataBytes[8];
  static const uint8_t txDLC = 5;
  
  dataBytes[0] = txUnitID[0]; // set unit ID
  dataBytes[1] = txUnitID[1]; // set unit ID
  dataBytes[2] = txUnitID[2]; // set unit ID
  dataBytes[3] = txUnitID[3]; // set unit ID
  dataBytes[4] = (txSwitchID); // set switch ID
  

  switch (swState) {

  case 0: // switch off
    send_message(SW_SET_OFF_ID, dataBytes, txDLC);
    break;
  case 1: // switch on
    send_message(SW_SET_ON_ID, dataBytes, txDLC);
    break;
  case 2: // momentary press
    send_message(SW_MOM_PRESS_ID, dataBytes, txDLC);
    break;
  default: // unsupported state
    Serial.println("Invalid switch state for transmission");
    break;
  }
}


static void setSwitchState(uint8_t *data, uint8_t swState) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  uint16_t switchID = (data[4] << 8) | data[5]; // switch ID
  uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  
  switch (swState) {
    case 0: // switch off
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d OFF\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    case 1: // switch on
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d ON\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    case 2: // momentary press
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d MOMENTARY PRESS\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    default:
      Serial.println("Invalid switch state");
      break;
  }
}

/**
 * @brief Helper to configure a standard digital output submodule
 * @param idx The submodule index (0-7)
 * @param pin The GPIO pin number
 * @param mode Initial output mode or state
 */
void setupAnalogStripDigitalOut(uint8_t idx, uint8_t pin, uint8_t mode) {
  if (idx >= MAX_SUB_MODULES) return; /* Safety check against array bounds */

  subModule_t& sub = node.subModule[idx];
  sub.introMsgId  = OUT_GPIO_DIGITAL_ID;
  sub.introMsgDLC = OUT_GPIO_DIGITAL_DLC;
  sub.dataMsgId   = SET_ANALOG_STRIP_COLOR_ID;
  sub.dataMsgDLC  = SET_ANALOG_STRIP_COLOR_DLC;
  sub.saveState   = true;
  sub.config.digitalOutput.outputPin  = pin;
  sub.config.digitalOutput.outputMode = mode;
}

/**
 * @brief Helper to configure a standard LCD Touchscreen submodule
 * @param idx The submodule index (0-7)
 */
void setupLcdTouchscreen(uint8_t idx) {
  if (idx >= MAX_SUB_MODULES) return;

  subModule_t& sub = node.subModule[idx];
  sub.introMsgId  = DISP_TOUCHSCREEN_LCD_ID;
  sub.introMsgDLC = DISP_TOUCHSCREEN_LCD_DLC;
  sub.dataMsgId   = DATA_TOUCHSCREEN_EVENT_ID;
  sub.dataMsgDLC  = DATA_TOUCHSCREEN_EVENT_DLC;
  sub.saveState   = true;
}

/**
 * @brief Helper to configure an Analog Backlight submodule
 * @param idx The submodule index (0-7)
 * @param pin The GPIO pin for PWM/Control
 */
void setupAnalogBacklight(uint8_t idx, uint8_t pin) {
  if (idx >= MAX_SUB_MODULES) return;

  subModule_t& sub = node.subModule[idx];
  sub.introMsgId  = DISP_ANALOG_BACKLIGHT_ID;
  sub.introMsgDLC = DISP_ANALOG_BACKLIGHT_DLC;
  sub.dataMsgId   = DATA_DISPLAY_MODE_ID;
  sub.dataMsgDLC  = DATA_DISPLAY_MODE_DLC;
  sub.saveState   = true;
  sub.config.digitalOutput.outputPin  = pin;
  sub.config.digitalOutput.outputMode = OUT_MODE_TOGGLE;
}

/**
 * @brief Helper to configure an ARGB LED Strip submodule
 * @param idx The submodule index (0-7)
 * @param pin The GPIO pin for the data signal
 * @param count Number of LEDs in the strip
 */
void setupArgbStrip(uint8_t idx, uint8_t pin, uint16_t count) {
  if (idx >= MAX_SUB_MODULES) return;

  subModule_t& sub = node.subModule[idx];
  sub.introMsgId  = DISP_ARGB_LED_STRIP_ID;
  sub.introMsgDLC = DISP_ARGB_LED_STRIP_DLC;
  sub.dataMsgId   = SET_ARGB_STRIP_COLOR_ID;
  sub.dataMsgDLC  = SET_ARGB_STRIP_COLOR_DLC;
  sub.saveState   = true;
  sub.config.argbLed.outputPin = pin;
  sub.config.argbLed.ledCount  = count;
}

/**
 * @brief Helper to configure a Digital Input submodule
 * @param idx The submodule index (0-7)
 * @param pin The GPIO pin for the input
 * @param res Internal resistor mode (PULLUP, PULLDOWN, etc.)
 */
void setupDigitalInput(uint8_t idx, uint8_t pin, uint8_t res) {
  if (idx >= MAX_SUB_MODULES) return;

  subModule_t& sub = node.subModule[idx];
  sub.introMsgId  = INPUT_DIGITAL_GPIO_ID;
  sub.introMsgDLC = INPUT_DIGITAL_GPIO_DLC;
  sub.dataMsgId   = DATA_BUTTON_DOWN_ID;
  sub.dataMsgDLC  = DATA_BUTTON_DOWN_DLC;
  sub.saveState   = false;
  sub.config.digitalInput.inputPin = pin;
  sub.config.digitalInput.outputRes = res;
}

/** Load CYD node info into the nodeInfo struct */
void nodeInfoCYD() { /* remote node type IFACE_TOUCHSCREEN_TYPE_A_ID 0x792 */
  node.nodeID      = unpackBytestoUint32((const uint8_t*)&myNodeID[0]);
  node.nodeTypeMsg = IFACE_TOUCHSCREEN_TYPE_A_ID;
  node.nodeTypeDLC = IFACE_TOUCHSCREEN_TYPE_A_DLC;
  node_crc         = CRC_INVALID_CONFIG; /* set the CRC to an invalid value indicating node has not received a configuration */

  /* Touchscreen and backlight: Handled via helper */
  setupLcdTouchscreen(0);
  setupAnalogBacklight(1, CYD_BACKLIGHT_PIN);

  /* RGB LED: Handled via helper */
  setupAnalogStripDigitalOut(2, CYD_LED_BLUE_PIN,  OUT_STATE_ON);  /* Index 2 */
  setupAnalogStripDigitalOut(3, CYD_LED_GREEN_PIN, OUT_STATE_ON);  /* Index 3 */
  setupAnalogStripDigitalOut(4, CYD_LED_RED_PIN,   OUT_STATE_OFF); /* Index 4 */
}


/**
 * @brief Load ARGB node info into the nodeInfo struct
 *
 * @details This function populates the nodeInfo struct with the
 *          necessary information for an ARGB node. This includes
 *          the node ID, node type message ID, node type DLC,
 *          and the CRC value. Additionally, this function sets
 *          up the sub modules associated with the node, which are
 *          the ARGB strip and a digital input.
 */
void nodeInfoARGB() { /* remote node type IFACE_ARGB_MULTI_ID 0x79C */
  node.nodeID      = unpackBytestoUint32((const uint8_t*)&myNodeID[0]);
  node.nodeTypeMsg = IFACE_ARGB_MULTI_ID;
  node.nodeTypeDLC = IFACE_ARGB_MULTI_DLC;
  node_crc         = CRC_INVALID_CONFIG;

  /* Configure via specialized helpers */
  setupArgbStrip(0, M5STAMP_ARGB_PIN, M5STAMP_ARGB_COUNT);
  setupDigitalInput(1, M5STAMP_BUTTON_PIN, INPUT_RES_PULLUP);
  
  /* node.subModCnt will be calculated dynamically during intro */
}

/** * @brief Prints a hex dump of a memory block to the Serial console.
 * @param ptr Pointer to the memory block
 * @param size Size of the block in bytes
 */
void printHexDump(const void* ptr, size_t size) {
    const uint8_t* p = (const uint8_t*)ptr;
    char buf[16]; /* Buffer for hex formatting */
    
    Serial.println("\n--- nodeInfo_t Hex Dump ---");
    for (size_t i = 0; i < size; i++) {
        /* Print address offset every 16 bytes */
        if (i % 16 == 0) {
            if (i > 0) Serial.println();
            Serial.printf("%04X: ", (uint16_t)i);
        }
        
        Serial.printf("%02X ", p[i]);
    }
    Serial.println("\n---------------------------");
}

static void loadDefaults(uint16_t nodeType) {
  switch (nodeType) {
      case (IFACE_ARGB_MULTI_ID): /* ARGB multi */
        nodeInfoARGB(); /* load node info */
        break;
      case (IFACE_TOUCHSCREEN_TYPE_A_ID): /* touchscreen type a */
        nodeInfoCYD(); /* load node info */
        break;

      default:
        nodeInfoARGB(); /* load node info */
        break;
  }
  printHexDump(&node, sizeof(node));
}

// static void sendIntroack() {
//   // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
//   send_message(ACK_INTRO_ID, (uint8_t *)myNodeID, ACK_INTRO_DLC);
// }

/**
 * @brief Get the current epoch time from the system clock.
 *
 * This function reads the current time from the ESP32 system clock
 * and returns it as a uint32_t representing the number of seconds
 * since the epoch (January 1, 1970, 00:00:00 UTC).
 *
 * @return uint32_t The current epoch time in seconds.
 */
static uint32_t getEpochTime() {
  /* Get time from the system clock and return it as a uint32_t */
  struct timespec newTime;

  clock_gettime(CLOCK_REALTIME, &newTime); /* Read time from ESP32 clock*/

  return (uint32_t)newTime.tv_sec; 
}

/**
 * @brief Send a uint32_t on the CAN bus
 * @param bigNumber The uint32_t to be sent
 * @param canMsgId The message ID to use for transmission (default: DATA_EPOCH_ID)
 * @param dlc The data length code to use for transmission (default: DATA_EPOCH_DLC)
 * 
 * This function takes a uint32_t and sends it on the CAN bus
 * with the given message ID and data length code.
 */
static void sendCanUint32(uint32_t bigNumber, uint32_t canMsgId = DATA_EPOCH_ID, uint8_t dlc = DATA_EPOCH_DLC) {
  /* Take a uint32_t and put it on the bus */
  uint8_t dataBytes[dlc];
  
  dataBytes[0] = myNodeID[0]; // set node ID
  dataBytes[1] = myNodeID[1]; // set node ID
  dataBytes[2] = myNodeID[2]; // set node ID
  dataBytes[3] = myNodeID[3]; // set node ID
  dataBytes[4] = (bigNumber >> 24) & 0xFF;
  dataBytes[5] = (bigNumber >> 16) & 0xFF;
  dataBytes[6] = (bigNumber >> 8) & 0xFF;
  dataBytes[7] = (bigNumber & 0xFF);

  send_message(canMsgId, (uint8_t *)dataBytes, dlc);

}
/**
 * @brief Receive time in seconds and write it to the ESP32 clock
 *
 * This function is used to receive time in seconds from the gateway node and write it to the ESP32 clock.
 * The time received is used to synchronize the ESP32 clock with the gateway node's clock.
 *
 * @param epochTime The time in seconds to be written to the ESP32 clock
 * @return None
 */
static void setEpochTime(uint32_t epochTime) {
/* Receive time in seconds and write it to the ESP32 clock */

  struct timespec newTime;
  newTime.tv_sec = (time_t)epochTime;
  newTime.tv_nsec = 0;
  clock_settime(CLOCK_REALTIME, &newTime);
  
}

/**
 * @brief Updates a specific LED strip based on received CAN color index.
 * @param ledIndex The submodule index (0-7).
 * @param colorIndex The 1-byte color index from the CAN message.
 */
void handleColorCommand(uint8_t ledIndex, uint8_t colorIndex) {
#ifdef ARGB_LED
    /* 1. Range check the submodule index */
    if (ledIndex >= 8) return;

    /* 2. Verify this submodule is actually an ARGB strip */
    if (node.subModule[ledIndex].introMsgId != DISP_ARGBW_LED_STRIP_ID) {
        return; /**< Not an ARGB module, ignore command */
    }

    /* 3. Range check the color index against your palette */
    if (colorIndex < 32) {
        CRGB targetColor = SystemPalette[colorIndex];
        
        /* Fetch the dynamic count from the configuration struct */
        uint16_t count = node.subModule[ledIndex].config.argbLed.ledCount;

        /** * 4. Update only the specific buffer for this submodule.
         * ledData[ledIndex] was assigned to FastLED during initHardware().
         */
        fill_solid(ledData[ledIndex], count, targetColor);
        
        FastLED.show();
        
        Serial.printf("Submod %u (ARGB): Updated to palette index %u\n", ledIndex, colorIndex);
    }
#endif
}

/**
 * @brief Handles the erase NVS command from the master node
 * 
 * This function attempts to erase the NVS configuration and
 * reset the node to its default configuration. If the erase
 * operation fails due to a mutex timeout, the function will
 * retry up to 3 times with a short delay in between retries.
 */
void handleEraseNVS() {
  ConfigStatus cfgStatus;
  int retries = 0;

  Serial.println("\nErasing config...");

  do {
      cfgStatus = eraseConfig(); 

      if (cfgStatus == CFG_OK) {
          Serial.println("Config erased successfully, rebooting...");
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before reboot */
          ESP.restart(); /**< Reboot the ESP32 */
          break; 
      } 
      
      if (cfgStatus == CFG_ERR_NOT_FOUND) {
          Serial.println("Config not found.");
          break; 
      } 

      if (cfgStatus == CFG_ERR_MUTEX) {
          Serial.printf("Flash busy - Retry %d/3...\n", retries + 1);
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
      }

  } while ((cfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (cfgStatus == CFG_ERR_MUTEX) {
      Serial.println("Critical Error: Could not access NVS (Mutex Timeout)");
  }
}

/**
 * @brief Attempts to load the configuration from NVS.
 *
 * This function attempts to load the configuration from NVS and
 * initializes the hardware if successful. If the configuration
 * is invalid or not found, it loads the default configuration from
 * the build flag node type and starts in PROVISIONING MODE. If the
 * function is unable to access NVS due to a mutex timeout, it
 * loads the default configuration from the build flag node type.
 */
void handleReadNVS() {
  ConfigStatus loadCfgStatus;
  int retries = 0;

  Serial.println("\nLoading config...");

  do {
      loadCfgStatus = loadConfig(node); 

      if (loadCfgStatus == CFG_OK) {
          Serial.println("Config loaded successfully.");
          FLAG_VALID_CONFIG = true;
          initHardware(); /**< Initialize the hardware */
          break; 
      } 
      
      if (loadCfgStatus == CFG_ERR_CRC || loadCfgStatus == CFG_ERR_NOT_FOUND) {
          Serial.println("Invalid config - Starting in PROVISIONING MODE");
          loadDefaults(NODEMSGID); /**< load defaults from build flag node type */
          break; 
      } 

      if (loadCfgStatus == CFG_ERR_MUTEX) {
          Serial.printf("Flash busy - Retry %d/3...\n", retries + 1);
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
      }

  } while ((loadCfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (loadCfgStatus == CFG_ERR_MUTEX) {
      Serial.println("Critical Error: Could not access NVS (Mutex Timeout)");
      loadDefaults(NODEMSGID); /* load defaults from build flag node type */
  }
}

/**
 * @brief Dynamically counts active submodules in the node structure
 * * Scans the subModule array and counts entries with a non-zero introMsgId.
 * This eliminates the need to hard-code subModCnt in the default config.
 * * @return uint8_t Total number of configured submodules
 */
uint8_t countActiveSubModules() {
    uint8_t count = 0;
    /* Iterate through the maximum possible submodules (8) */
    for (int i = 0; i < 8; i++) {
        if (node.subModule[i].introMsgId != 0) {
            count++;
        }
    }
    return count;
}

/**
 * @brief Send an introduction message about the node
 * 
 * This function is called in response to a request from the master node
 * to introduce the node. The function will send an introduction message
 * with the node type and sub-module count. The function will
 * also send an introduction message for each sub-module with the
 * sub-module type and configuration.
 * 
 * The function uses a cooldown to prevent spamming the bus every 100ms tick
 */
void sendIntroduction(int msgPtr = 0) {
  uint32_t currentTick = millis();
  uint16_t txMsgID  = 0;
  uint32_t txMsgDLC = 8U; /* Default to 8 bytes */
  uint8_t msgData[8];

  /* Update the count dynamically before processing the introduction */
  node.subModCnt = countActiveSubModules();

  memset(&msgData, 0, sizeof(msgData)); /* wipe the buffer before using it */

  /* Consistent 32-bit Node ID across all intro frames */
  packUint32ToBytes(node.nodeID, &msgData[0]);

/* 0: Node Identity */
  if (msgPtr == 0) {
    /* Check FLAG_VALID_CONFIG flag, if it's set use the in-memory CRC, if not use 0xFFFF */    
    uint16_t txCrc = FLAG_VALID_CONFIG ? getConfigurationCRC(node) : CRC_INVALID_CONFIG;
    // uint16_t txCrc = getConfigurationCRC(node);
    
    txMsgID    = node.nodeTypeMsg;
    msgData[4] = node.subModCnt;
    msgData[5] = (uint8_t)(txCrc >> 8);
    msgData[6] = (uint8_t)(txCrc);
    Serial.printf("TX INTRO: NODE 0x%08X SUBMOD %02u (Type: 0x%03X, CRC: 0x%04X)\n", node.nodeID, msgData[4], txMsgID, txCrc);
  } 
/* >0: Sub-module Identity (Part A and Part B) */
  else {
    uint8_t modIdx = (uint8_t)((msgPtr - 1) / 2); /**< Map ptr to sub-module index */
    bool isPartB   = ((msgPtr - 1) % 2) != 0;      /**< Alternate A/B sequence */
    
    if (modIdx >= node.subModCnt) return;
    subModule_t& sub = node.subModule[modIdx];

    txMsgID = sub.introMsgId;
    
    if (!isPartB) {
        /* Part A: Configuration Data */
        msgData[4] = modIdx; /**< bits 0-6: index, bit 7: 0 (Part A) */
        msgData[5] = sub.config.rawConfig[0];
        msgData[6] = sub.config.rawConfig[1];
        msgData[7] = sub.config.rawConfig[2];
    } else {
        /* Part B: Telemetry/Operational Data */
        msgData[4] = modIdx | 0x80; /**< Set bit 7 to indicate Part B */
        msgData[5] = (uint8_t)(sub.dataMsgId >> 8);
        msgData[6] = (uint8_t)(sub.dataMsgId);
        /* Pack DLC (4 bits) and SaveState (1 bit) into byte 7 */
        msgData[7] = (sub.dataMsgDLC & 0x0F) | (sub.saveState ? 0x80 : 0x00);
    }

    if (txMsgID == 0) {
        return;  /* error condition, msg ID is invalid, exit the routine */
    }

    Serial.printf("TX INTRO: MOD 0x%03X at Idx %i (Cfg: %02X %02X %02X)\n", 
                  txMsgID, msgData[4], msgData[5], msgData[6], msgData[7]);
  }
  /* put the message on the bus */
  send_message(txMsgID, msgData, txMsgDLC);

} /* end sendIntroduction */


static void rxProcessMessage(twai_message_t &message) {
  // twai_message_t altmessage;
  bool msgFlag = false;


  if (message.data_length_code > 0) { // message contains data, check if it is for us
    uint8_t rxUnitID[4] = {message.data[0], message.data[1], message.data[2], message.data[3]};
    // memcmp((const uint8_t *)rxUnitID, (const uint8_t *)myNodeID, 4);

    if (memcmp(message.data, (const uint8_t *)myNodeID, 4) == 0) {
      msgFlag = true; // message is for us
      // Serial.printf("Node ID matched for message id 0x%x\n", message.identifier);
    } else {
      msgFlag = false; // message is not for us     
      // sendIntroack();
      // Serial.printf("Overheard message 0x%03x for node %02x:%02x:%02x:%02x\n", message.identifier, rxUnitID[0], rxUnitID[1], rxUnitID[2], rxUnitID[3]);
    }
  } else {
    msgFlag = true; // general broadcast message is valid
    Serial.printf("RX BROADCAST MSG: 0x%x NO DATA\n", message.identifier);
  }

  if (!msgFlag) {
    // Serial.println("Message does not match our ID, end of process.");
    return;
  }

  /* debug: dump message data */
  Serial.printf("RX MSG: 0x%03X DATA: ", message.identifier);
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf("0x%02X ", message.data[i]);
  }
  Serial.println();

  switch (message.identifier) {
    case SW_SET_OFF_ID:            // set output switch off
      setSwitchState(message.data, 0);
      txSwitchState((uint8_t *)myNodeID, 32, 2); 
      break;
    case SW_SET_ON_ID:             // set output switch on
      setSwitchState(message.data, 1);
      txSwitchState((uint8_t *)myNodeID, 32, 0); 
      break;
    case SW_SET_MODE_ID:           // setup output switch modes
      setSwitchMode(message.data);
      break;
    case SW_SET_BLINK_DELAY_ID:          // set output switch blink delay
      setSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT_ID:          // set output switch strobe pattern
      setSwStrobePat(message.data);
      break;
    case SET_DISPLAY_OFF_ID:          // set display off
      setDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON_ID:          // set display on
      setDisplayMode(message.data, 1); 
      break;    
    case SET_ARGB_STRIP_COLOR_ID:          /* set ARGB color */
      handleColorCommand(message.data[4], message.data[5]); /* byte 4 is the sub module index, byte 5 is the color index */
      break;
    case CFG_ARGB_STRIP_ID:                                                     /* setup ARGB channel */
      {
      uint8_t modIdx = message.data[4];                                         /* byte 4 holds the sub module index */
      node.subModule[modIdx].config.argbLed.outputPin  = message.data[5];        /* byte 5 holds the output pin */
      node.subModule[modIdx].config.argbLed.ledCount   = message.data[6];         /* bytes 6 hold the number of LEDs (max 255)*/
      node.subModule[modIdx].config.argbLed.colorOrder = message.data[7];       /* byte 7 holds the color order */
      }
      break;
    case CFG_DIGITAL_INPUT_ID: /**< Setup digital input channel */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.digitalInput.inputPin   = message.data[5];
        node.subModule[modIdx].config.digitalInput.outputRes  = message.data[6];
        node.subModule[modIdx].config.digitalInput.isInverted = message.data[7];
      }
      break;

    case CFG_ANALOG_INPUT_ID: /**< Setup analog ADC input channel */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.analogInput.inputPin = message.data[5];
        /** bytes 6 and 7 hold the 16-bit oversampling count */
        node.subModule[modIdx].config.analogInput.overSampleCnt = (message.data[6] << 8) | message.data[7];
      }
      break;

    case CFG_DIGITAL_OUTPUT_ID: /**< Setup digital output channel (relays/mosfets) */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.digitalOutput.outputPin   = message.data[5];
        node.subModule[modIdx].config.digitalOutput.momPressDur = message.data[6];
        node.subModule[modIdx].config.digitalOutput.outputMode  = message.data[7];
      }
      break;

    case CFG_PWM_OUTPUT_ID: /**< Setup PWM output channel */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.pwmOutput.outputPin  = message.data[5];
        node.subModule[modIdx].config.pwmOutput.pwmFreq    = message.data[6];
        node.subModule[modIdx].config.pwmOutput.isInverted = message.data[7];
      }
      break;

    case CFG_BLINK_OUTPUT_ID: /**< Setup blinking/strobing output channel */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.blinkOutput.outputPin  = message.data[5];
        node.subModule[modIdx].config.blinkOutput.blinkDelay = message.data[6];
        node.subModule[modIdx].config.blinkOutput.strobePat  = message.data[7];
      }
      break;

    case CFG_ANALOG_STRIP_ID: /**< Setup analog RGB/RGBW strip */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.analogStrip.stripIndex = message.data[5];
        node.subModule[modIdx].config.analogStrip.colorIndex = message.data[6];
        node.subModule[modIdx].config.analogStrip.pinIndex   = message.data[7];
      }
      break;

    case CFG_ANALOG_OUTPUT_ID: /**< Setup analog DAC output channel */
      {
        uint8_t modIdx = message.data[4];
        node.subModule[modIdx].config.analogOutput.outputPin  = message.data[5];
        node.subModule[modIdx].config.analogOutput.outputMode = message.data[6];
        /** message.data[7] is reserved/padding */
      }
      break;      
    case CFG_ERASE_NVS_ID: /**< 0x41B: Master requesting NVS erase */
      {
        Serial.println("Master requesting NVS erase...");
        handleEraseNVS();
      }
      break;
    case CFG_REBOOT_ID: /**< 0x41C: Master requesting reboot */
      {
        Serial.println("Master requested reboot, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before reboot */
        ESP.restart();
      }
      break;
    case CFG_WRITE_NVS_ID: /**< 0x41D: Master requesting NVS commit */
      {

        uint16_t masterCrc = (message.data[4] << 8) | message.data[5];
        uint16_t localCrc  = getConfigurationCRC(node);
        
        uint8_t responseData[6];
        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(node.nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8);
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        if (masterCrc == localCrc) {
            /** CRCs match, attempt to persist to flash */
            if (saveConfig(node) == CFG_OK) {
                /** SUCCESS: Set the flag indicating config is valid and send back the verified CRC */
                FLAG_VALID_CONFIG = true;
                send_message(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
                Serial.println("NVS Commit Successful");
            } else {
                /** Flash hardware error */
                send_message(DATA_CFGWRITE_FAILED_ID, responseData, DATA_CFGWRITE_FAILED_DLC);
                Serial.println("NVS Commit Failed: Flash Error");
            }
        } else {
            /** CRC Mismatch: Provisioning corrupted or incomplete */
            send_message(DATA_CFGWRITE_FAILED_ID, responseData, DATA_CFGWRITE_FAILED_DLC);
            Serial.printf("NVS Commit Failed: CRC Mismatch (M:%04X L:%04X)\n", masterCrc, localCrc);
        }
      }
      break;    
    case CFG_READ_NVS_ID: /**< 0x41E: Master requesting NVS read */
      {
        Serial.println("NVS Read Request");
        uint16_t masterCrc = (message.data[4] << 8) | message.data[5];
        
        handleReadNVS();

        uint16_t localCrc  = getConfigurationCRC(node);
        uint8_t responseData[6];

        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(node.nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8);
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        send_message(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
      }
    case REQ_NODE_INTRO_ID:
      Serial.println("Interface intro request, responding with our introduction");
      // FLAG_SEND_INTRODUCTION = true; /* set flag to send introduction message */
      introMsgPtr = 0;               /* reset introduction message pointer */
      sendIntroduction(introMsgPtr); /* send the introduction message immediately */
      break;
    case ACK_INTRO_ID:
      Serial.println("Received introduction acknowledgement, advance pointer");
      introMsgPtr++;
      /* Sequence length is now 1 (node) + 2 messages per submodule */
      if (introMsgPtr > (node.subModCnt * 2)) {
          introMsgPtr = 0;
      }
      sendIntroduction(introMsgPtr);
      break;
    case DATA_EPOCH_ID:
      // Use explicit casting to prevent shift overflow
      uint32_t epochTime;
      epochTime = ((uint32_t)message.data[4] << 24) | 
                  ((uint32_t)message.data[5] << 16) | 
                  ((uint32_t)message.data[6] << 8)  | 
                   (uint32_t)message.data[7];
      setEpochTime((uint32_t)epochTime);
      Serial.println("Received epoch from master; updating clock");
      break;

    default:
      Serial.printf("Unknown message received 0x%x\n", message.identifier);
      // sendIntroack();
      break;
  }
} // end of handle_rx_message

/**
 * @brief Validates and sorts messages that passed the broad hardware filter
 * @param msg The TWAI message frame received from the bus
 */
void handleCanRX(twai_message_t& msg) {
  /* Extract the "Base" of the ID by masking out the lower 6 bits (0x3F) */
  uint16_t idBase = msg.identifier & ~0x3F;

  /* Software filter: only log introductions for remote ARGB nodes (for the CYD) */
#ifdef ESP32CYD
  if (msg.identifier == 0x701 || msg.identifier == 0x702 || msg.identifier == 0x711) {
      if (msg.data_length_code >= 4) { /* Need at least 4 bytes to proceed */
          uint32_t remoteNodeId;
          /* Use bytes 0-3 to match your sendIntroduction format */
          remoteNodeId = ((uint32_t)msg.data[0] << 24) | 
                          ((uint32_t)msg.data[1] << 16) | 
                          ((uint32_t)msg.data[2] << 8)  | 
                          (uint32_t)msg.data[3];

          if (remoteNodeId != 0) { /* Sanity check, only register if node ID is non-zero */
              registerARGBNode(remoteNodeId);
          } 
      }
      return;
    }
#endif   

    /* Pass everything else onto rxProcessMessage */
    rxProcessMessage(msg);
}


/**
 * @brief Manages periodic transmissions in TaskTWAI
 */
void managePeriodicMessages() {
    static uint32_t lastHeartbeat = 0;
    static uint32_t lastIntro = 0;
    uint32_t currentMillis = millis();

    /** Introduction as Heartbeat - Every 10 Seconds 
        We send this as the heartbeat to save bandwidth,
        unless FLAG_SEND_INTRODUCTION is manually set by a Master request. */
    if (currentMillis - lastIntro >= 10000) {
        lastIntro = currentMillis;
        Serial.printf("Sending heartbeat (ptr = %i)\n", introMsgPtr);
        sendIntroduction(0); /* send the node introduction message as heartbeat */

        /* If we are stuck mid-sequence, reset after 10s of silence */
        if (introMsgPtr != 0) {
            Serial.println("Intro sequence timed out, resetting pointer.");
            introMsgPtr = 0;
        }
    }
}

void TaskTWAI(void *pvParameters) {
  // give some time at boot for the cpu setup other parameters
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL); 
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); /* accept all messages, filter in software */

  /* Install TWAI driver */
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI installed");
  } else {
    Serial.println("Failed to install TWAI");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Start TWAI driver */
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI started");
  } else {
    Serial.println("Failed to start TWAI");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states */
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("TWAI alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts"); 
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* TWAI driver is now successfully installed and started */
  can_driver_installed = true;
  // FLAG_SEND_INTRODUCTION = true; /* send an introduction message */
  introMsgPtr = 0;  /* reset the intro message pointer */
  sendIntroduction(0); /* send the first introduction message */
  int loopCount = 0;

  /** Begin main can bus RX/TX loop */
  for (;;) { 
    if (!can_driver_installed || can_suspended) {
      /* Driver not installed or bus suspended */
      vTaskDelay(pdMS_TO_TICKS(100)); /* Idle the task */
      continue; /* Skip the rest of the loop */
    }
    /* Check if alert happened */
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    /* Timestamps for throttling Serial output (static to persist across loops) */
    static uint32_t lastErrPassLog = 0;
    static uint32_t lastBusErrLog = 0;
    static uint32_t lastTxFailLog = 0;
    static uint32_t lastRxFullLog = 0;
    const uint32_t LOG_INTERVAL = 2000; /* Log once every 2 seconds max */

    /* Handle alerts */
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      if (millis() - lastErrPassLog > LOG_INTERVAL) {
        Serial.println("Alert: TWAI controller has become error passive.");
        lastErrPassLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      if (millis() - lastBusErrLog > LOG_INTERVAL) {
        Serial.printf("Alert: Bus Error. Count: %d\n", twaistatus.bus_error_count);
        lastBusErrLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      if (millis() - lastTxFailLog > LOG_INTERVAL) {
        Serial.println("Alert: Transmission failed.");
        lastTxFailLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      if (millis() - lastRxFullLog > LOG_INTERVAL) {
        Serial.println("Alert: RX queue full.");
        lastRxFullLog = millis();
      }
    }

    /* Check if message is received */
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
#ifdef ESP32CYD
      digitalWrite(LED_BLUE, LOW); /* Turn blue LED on (inverse logic) */
#endif
      /* One or more messages received. Handle all. */
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handleCanRX(message); 
#ifdef ESP32CYD
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE)); /* Toggle blue LED */
#endif
      }
    }

    /* Send periodic messages */
    managePeriodicMessages();

    vTaskDelay(10);
#ifdef ESP32CYD
    digitalWrite(LED_BLUE, HIGH); /* Turn blue LED off (inverse logic) */
#endif

  }
}

void setup() {

#ifdef ESP32CYD
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_BLUE, HIGH); /* reverse logic, high equals off */
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
#endif

  memset(&node, 0, sizeof(nodeInfo_t)); /**< Clear the struct */

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(2500); /* Provide time for the board's usb interface to change from flash to uart mode */

  /* Debug check for memory alignment */
  Serial.printf("\nStruct Sizes - nodeInfo_t: %d, subModule_t: %d\n", 
              sizeof(nodeInfo_t), sizeof(subModule_t));

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.begin(ssid, password);
  
  /* set up some clock parameters */
  setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
  tzset();

  readMacAddress(); /**< Read the ESP32 station MAC address and program myNodeID */

  /* Node Setup Logic */
  memset(&node, 0, sizeof(nodeInfo_t)); /* Clear the struct */
  handleReadNVS(); /* Read the NVS data from flash and init hardware */


  #ifdef ESP32CYD
  initCYD(); /* Initialize CYD interface */
  #endif

  /* Start the CAN task */
  xTaskCreate(
    TaskTWAI,     /* Task function */
    "Task TWAI",  /*  name of task */
    4096,         /* Stack size of task */
    NULL,         /* parameter of the task */
    2,            /* priority of the task */
    &xTWAIHandle  /* Task handle to keep track of created task */
  );              

  /* Start OTA task  */
  xTaskCreate(
    TaskOTA,
    "Task OTA",
    4096,   
    NULL,
    4,
    NULL
  );
}

void printWifi() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}



void loop() {
 
  // NOP;
}