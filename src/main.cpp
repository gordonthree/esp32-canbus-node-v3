/**
 * @file main.cpp
 * @brief Gateway controller for CAN-based ARGB and Vehicle Control system.
 * @details Handles Wi-Fi/OTA, TWAI (CAN) hardware lifecycle, and node discovery.
 * Acts as the bridge between the CYD UI and the distributed hardware nodes.
 * * @note Acceptance Filters:
 * - Filter 1: 0x200-0x23F (Control/Interface)
 * - Filter 2: 0x400-0x43F & 0x700-0x73F (Telemetry/ARGB)
 */
#include <Arduino.h>
#include <ArduinoOTA.h>

#include <stdio.h>

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
#if defined(ARGB_LED) || defined(ESP32CYD)
#include "colorpalette.h"
#endif

struct canNodeInfo nodeInfo; /**< Store information about this node */


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

#define CAN_MY_IFACE_TYPE (0x701U) /* ARGB LED */
#define CAN_SELF_MSG 1


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



/* CAN bus stuff: */
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000
volatile bool can_suspended = false;
volatile bool can_driver_installed = false;

unsigned long previousMillis = 0;  /* will store last time a message was sent */
String texto;
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
// setup the ARGB led for Fastled
#ifndef ARGB_NUM_LEDS
#define ARGB_NUM_LEDS 1
#endif

#ifndef ARGB_DATA_PIN
#define ARGB_DATA_PIN 27
#endif

CRGB leds[ARGB_NUM_LEDS];
#endif

unsigned long ota_progress_millis = 0;

volatile bool wifi_connected = false;
volatile uint8_t myNodeID[4]; /**< node ID comprised of four bytes from MAC address */ 

void IRAM_ATTR Timer0_ISR()
{
  isrFlag = true;
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
void setupNodeInfo(const uint32_t nodeType) {
  nodeInfo.nodeID         = packBytes32((const uint8_t*)myNodeID); /**< convert node id string into a uint32 */

  switch (nodeType) {
    case (IFACE_ARGB_MULTI_ID):
      nodeInfo.nodeTypeMsg    = IFACE_ARGB_MULTI_ID;
      nodeInfo.nodeTypeDLC    = IFACE_ARGB_MULTI_DLC;
      nodeInfo.featureMask[0] = 0; /* no features */
      nodeInfo.featureMask[1] = 0; /* no features */
      nodeInfo.subModCnt = 2;

      nodeInfo.subModules[0].modType         = INPUT_ANALOG_KNOB_ID;
      nodeInfo.subModules[0].dataMsgId       = DATA_ANALOG_KNOB1_ID;
      nodeInfo.subModules[0].useFeatureMask  = false;

      nodeInfo.subModules[1].modType         = NODE_CPU_TEMP_ID;
      nodeInfo.subModules[1].dataMsgId       = DATA_NODE_CPU_TEMP_ID;
      nodeInfo.subModules[1].useFeatureMask  = false;
  }
  /* Initialize nodeInfo */
  
}

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

/**
 * @brief Updates LED strip based on received CAN color index
 * @param index The 1-byte color index from the CAN message
 */
void handleColorCommand(uint8_t ledIndex, uint8_t colorIndex) {
#ifdef ARGB_LED
    if (colorIndex < 32) {
        CRGB targetColor = SystemPalette[colorIndex];
        fill_solid(leds, ARGB_NUM_LEDS, targetColor);
        FastLED.show();
        Serial.printf("Color for LED array %u updated to index %u\n", ledIndex, colorIndex);
    }
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

#ifndef NODE_ID
#define NODE_ID BOX_MULTI_IO_ID // default node ID
#define NODE_DLC BOX_MULTI_IO_DLC
#endif

/**
 * @brief Send an introduction message to the gateway node with the node's ID and feature mask.
 *
 * This function is used to send the node's ID and feature mask to the gateway node.
 * The feature mask is used to indicate which features the node supports.
 *
 * @param None
 * @return None
 */
static void sendIntroduction() {
  uint8_t dataBytes[NODE_DLC];
  dataBytes[0] = myNodeID[0]; // set node ID
  dataBytes[1] = myNodeID[1]; // set node ID
  dataBytes[2] = myNodeID[2]; // set node ID
  dataBytes[3] = myNodeID[3]; // set node ID
  dataBytes[4] = (0x0F);      // display id
  dataBytes[5] = (0xA0);      // feature mask 0
  dataBytes[6] = (0xB0);      // feature mask 1

  send_message(NODE_ID, (uint8_t *)dataBytes, NODE_DLC); /**< send introduction message to the gateway node with the node's ID and feature mask */

}

static void sendIntroack() {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(ACK_INTRO_ID, (uint8_t *)myNodeID, ACK_INTRO_DLC);
}

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
    Serial.printf("RX MSG: 0x%x NO DATA\n", message.identifier);
  }

  /*   
  if (msgFlag == false) {
    return; // message is not for us, exit function
  }
 */
  if (!msgFlag) {
    // Serial.println("Message does not match our ID, end of process.");
    return;
  }

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
      handleColorCommand(0, message.data[5]); /* byte 4 is the display or led array index, byte 5 is the color index */
      break;
    case REQ_NODE_INTRO_ID:
      Serial.println("Interface intro request, responding with 0x702");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      sendIntroduction(); // send our introduction message
      break;
    case ACK_INTRO_ID:
      Serial.println("Received introduction acknowledgement, clearing flag");    
      FLAG_SEND_INTRODUCTION = false; // stop sending introduction messages
      txSwitchState((uint8_t *)myNodeID, 32, 1); 
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
    /* 1. Extract the "Base" of the ID by masking out the lower 6 bits (0x3F) */
    uint16_t idBase = msg.identifier & ~0x3F;

    /* 2. Software Filter: Only enter if it matches our three target ranges */
    switch (idBase) {
        case MSG_CTRL_IFACE: /* 0x200 */
            /* Handle Switch/Input Messages */
            rxProcessMessage(msg);
            break;

        case MSG_REQ_INTRO: /* 0x400 */
            /* Handle Status/Telemetry Messages */
            rxProcessMessage(msg);
            break;

        case MODULE_DISPLAY: /* 0x700 */
            #ifdef ESP32CYD
            if (msg.identifier == 0x701 || msg.identifier == 0x702 || msg.identifier == 0x711) {
                if (msg.data_length_code >= 4) { /**< Changed from 8 to 4 to be more lenient */
                    uint32_t remoteNodeId;
                    /* Use bytes 0-3 to match your sendIntroduction format */
                    remoteNodeId = ((uint32_t)msg.data[0] << 24) | 
                                   ((uint32_t)msg.data[1] << 16) | 
                                   ((uint32_t)msg.data[2] << 8)  | 
                                   (uint32_t)msg.data[3];

                    if (remoteNodeId != 0) {
                        registerARGBNode(remoteNodeId);
                    } else {
                        Serial.println("RX: ARGB ID was 0, ignoring.");
                    }
                }
            }
            #endif   
            break;

        default:
            /** * DISCARD "leaked" messages (e.g., 0x500 or 0x600) 
             * that passed the hardware filter but isn't for us.
             */
            return;
    }
}

void TaskTWAI(void *pvParameters) {
  // give some time at boot for the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);


  /** Range 1: 0x200 to 0x23F (Binary: 010 0000 0000 to 010 0011 1111)
   * Range 2: 0x400 to 0x43F (Binary: 100 0000 0000 to 100 0011 1111)
   */

  /* * Acceptance Code: Defines the bits that must match.
  * Code 1 (High 16 bits): 0x4000 (ID 0x200 shifted for SJA1000)
  * Code 2 (Low 16 bits):  0x8000 (ID 0x400 shifted for SJA1000)
  */
  // uint32_t acc_code = 0x40008000;

  /* * Acceptance Mask: Defines "don't care" bits.
  * For the SJA1000, a '1' in the mask means "don't care".
  * We want to ignore the lower 6 bits of the ID.
  */
  // uint32_t acc_mask = 0x07F807F8; 

  // twai_filter_config_t f_config = {
  //     .acceptance_code = acc_code,
  //     .acceptance_mask = acc_mask,
  //     .single_filter = false /* false = Dual Filter Mode */
  // };

  /** Hardware filter configuration for two ID ranges.
  * Range 1: 0x200 - 0x23F
  * Range 2: 0x400 - 0x43F
  */
  twai_filter_config_t f_config;
  f_config.single_filter = false; /* Enable Dual Filter Mode */

  /** Filter 1:
  * Code: Base ID shifted left 5 bits.
  * Mask: The bits we want to ignore (0x3F) shifted left 5, 
  * OR'd with the lower 5 bits (0x1F) which are used for 
  * flags like RTR that we also want to ignore here.
  */
  uint32_t code1 = (0x200 << 5);
  uint32_t mask1 = (0x03F << 5) | 0x1F;

  /** Filter 2:
  * Same logic for the 0x400 range.
  */
  // uint32_t code2 = (0x400 << 5);
  // uint32_t mask2 = (0x03F << 5) | 0x1F;

  /** Filter 2: Expanded to cover 0x400 AND 0x700 ranges
   * Base Code: 0x400
   * Mask: Ignore 0x300 (bits 8 and 9) + Ignore 0x03F (bits 0-5)
   * This effectively accepts anything matching 0x400, 0x500, 0x600, or 0x700 
   * that ends in the 0x00-0x3F range.
   */
  uint32_t code2 = (0x400 << 5); 
  uint32_t mask2 = (0x33F << 5) | 0x1F;

  /** Combine into the 32-bit acceptance registers.
  * Filter 1 occupies the high 16 bits, Filter 2 the low 16 bits.
  */
  f_config.acceptance_code = (code1 << 16) | code2;
  f_config.acceptance_mask = (mask1 << 16) | mask2;

  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.

  /* just accept all messages for debugging purposes */
  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // twai_filter_config_t 

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
  FLAG_SEND_INTRODUCTION = true; /* send an introduction message */
  int loopCount = 0;

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
      /* One or more messages received. Handle all. */
#ifdef ESP32CYD
      digitalWrite(LED_BLUE, LOW); /* Turn blue LED on (inverse logic) */
#endif
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handleCanRX(message); 
#ifdef ESP32CYD
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE)); /* Toggle blue LED */
#endif
      }
    }
    /* Send message */
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
      loopCount++;
      previousMillis = currentMillis;
      if (FLAG_SEND_INTRODUCTION) {
        FLAG_SEND_INTRODUCTION = false; /* Reset flag */
        sendIntroduction(); /* Send introduction message */
      }
      if (loopCount >= 10) {
        sendCanUint32(getEpochTime(), DATA_EPOCH_ID, DATA_EPOCH_DLC); /* Send epoch time as a heartbeat */
        FLAG_SEND_INTRODUCTION = true; /* Reset flag */
        loopCount = 0;
      }
      // send_message(REQ_NODE_INTRO_ID, NULL, REQ_NODE_INTRO_DLC); // send our introduction request
    }
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

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(2500); /* Provide time for the board's usb interface to change from flash to uart mode */

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.begin(ssid, password);
  delay(1000);

  Serial.println("AP Started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IPv4: ");
  Serial.println(WiFi.softAPIP());
  
  /* set up some clock parameters */
  setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
  tzset();

  readMacAddress(); /**< Read the ESP32 station MAC address and program myNodeID */

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
    &xTWAIHandle   /* Task handle to keep track of created task */
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

#ifdef ARGB_LED
  FastLED.addLeds<SK6812, ARGB_DATA_PIN, GRB>(leds, ARGB_NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
#endif



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