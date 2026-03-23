/**
 * @file main.cpp
 * @brief Gateway controller for CAN-based ARGB and Vehicle Control system.
 * @details Handles Wi-Fi/OTA, TWAI (CAN) hardware lifecycle, and node discovery.
 * Acts as the bridge between the CYD UI and the distributed hardware nodes.
 * Additional roles: ARGB LED control
 * Planned roles: Interface with i2c and SPI sensors, LCD and OLED displays, and button / keypad input
 *
 * @author Gordon McLellan
 */

 /* === Framework includes === */
#include <Arduino.h>
#include <ArduinoOTA.h>

/* === ESP32 includes === */
#include <Preferences.h>
#include <rom/crc.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <time.h>

/* === Standard library includes === */
#include <stdio.h>
#include <stddef.h>

/* === ESP32 native IDF includes === */
#include "driver/twai.h" /* esp32 native TWAI CAN library */
#include "driver/ledc.h" /* esp32 native LEDC PWM library */
#include "driver/gpio.h" /* esp32 native GPIO library */
#include "driver/adc.h"  /* esp32 native ADC library */
#include "esp_err.h"     /* esp32 error handler */



/* === FreeRTOS includes === */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef ARGB_LED
/* Load NeoPixelBus */
#include <NeoPixelBus.h>
#endif

/* === Local includes === */
#include "canbus_project.h"    /**< my various CAN functions and structs */
#include "secrets.h"           /**< WiFi credentials and such */
#include "can_router.h"        /**< CAN routing (subcriber) routines and constants */
#include "can_producer.h"      /**< CAN producer routines and constants */
#include "can_platform.h"      /**< CAN platform routines and constants */
#include "personality_table.h" /**< Node and sub-module personality table */
#include "node_state.h"        /**< Node and sub-module state table */
#include "storage.h"           /**< NVS storage routines */
#include "submodule_types.h"   /**< Sub-module type definitions */
#include "isr_gpio.h"          /**< GPIO interrupt routines */
#include "strobe_patterns.h"   /**< Strobe pattern definitions from can_personality */

/* my byte routines */
#include "byte_conversion.h"

/* hardware constants */
#include "esp32_defs.h"

#ifdef ESP32CYD
#include "espcyd.h"
#endif

/* my colors */
#if defined(ARGB_LED) || defined(ESP32CYD) || defined(ARGBW_LED)
#include "colorpalette.h"
#endif

/* memory allocation for main tasks */
#define TASK_TWAI_STACK_SIZE   (4096U)
#define TASK_OTA_STACK_SIZE    (4096U)
#define TASK_OUTPUT_STACK_SIZE (4096U)
#define TASK_INPUT_STACK_SIZE  (4096U)

#define tskLowPriority         (tskIDLE_PRIORITY + 1)
#define tskNormalPriority      (tskIDLE_PRIORITY + 2)
#define tskHighPriority        (tskIDLE_PRIORITY + 4)

#define CAN_ID_MASK            (0x3F)      /**< Mask for lower 6 bits of CAN ID */
#define PWM_RES_BITS           (8U)        /**< 8-bit resolution for PWM */
#define DEFAULT_TIMEZONE       "EST5EDT,M3.2.0,M11.1.0"
#define ENV_VAL_OVERWRITE      (1U)

#define CRC_INVALID_CONFIG     (0xFFFF)
#define SUBMOD_PART_B_FLAG     (0x80U)

/* Default CAN transceiver pins */
#ifndef RX_PIN
#define RX_PIN 22
#endif

#ifndef TX_PIN
#define TX_PIN 21
#endif

#ifndef NODEMSGID
#define NODEMSGID 0x701
#endif

/* CAN bus polling intervals */
#define TRANSMIT_RATE_MS (100U)
#define POLLING_RATE_MS  (100U)

/* WiFi Constants */
#define AP_SSID  "canesp32"

/* LEDC constants */
#define LEDC_CHANNELS_PER_TIMER (8U)   /**< ESP32: 8 channels per timer */
// #define LEDC_MAX_TIMERS         (8U)   /**< ESP32: 8 timers per channel */



/* Connect node state functions to producer library callback table */
static const producerCallbacks_t producerCB = {
    .getSubModuleCount = nodeGetSubModuleCount,
    .getSubModule      = nodeGetSubModule,
    .getRuntime        = nodeGetRuntime
};

// extern uint16_t crc16_ccitt(const uint8_t* data, uint16_t length);


outputTracker_t trackers[MAX_SUB_MODULES];

struct blinkerTracker_t {
    uint32_t     freq;              /**< LEDC frequency in Hertz */
    uint8_t      hwPin;             /**< Hardware pin */
    uint8_t      subIdx;            /**< Node submodule index */
    bool         isActive;          /**< Flag to indicate blinker is outputting a signal */
};

blinkerTracker_t blinkers[LEDC_MAX_TIMERS];
uint8_t pwmPins[LEDC_MAX_TIMERS];        /**< Array to track PWM pins */

SemaphoreHandle_t flashMutex = xSemaphoreCreateMutex(); /**< mutex for flash safety */

/* Task handles */
TaskHandle_t xTWAIHandle   = NULL; /* declared and defined */
TaskHandle_t xOutputHandle = NULL; /**< Handle for the output switch logic task */
TaskHandle_t xInputHandle  = NULL; /**< Handle for the input event logic task */

/* definitions from external libraries */
#ifdef ESP32CYD
// extern int discoveredNodeCount; /**< Track active count in the array */
// extern const int MAX_ARGB_NODES;
// extern ARGBNode discoveredNodes[];
#endif

/** Runtime data storage */
submoduleRuntime_t g_subRuntime[MAX_SUB_MODULES];

/* memory allocation for the flags */
uint8_t FLAG_SEND_INTRODUCTION = 0;
uint8_t FLAG_BEGIN_NORMAL_OPER = 0;
uint8_t FLAG_HALT_NORMAL_OPER  = 0;
uint8_t FLAG_SEND_HEALTHCHECK  = 0;
uint8_t FLAG_SEND_NODECHECK    = 0;
uint8_t FLAG_PRINT_TIMESTAMP   = 0;

/* OTA task control */
volatile bool ota_enabled  = false;
volatile bool ota_started  = false;
const char*   ota_password = OTA_PASSWORD; // change this

/* dynamic discovery stuff */
nodeInfo_t node; /**< Store information about this node */

volatile uint16_t node_crc = 0xffff; /**< CRC-16 for the node configuration */
volatile int introMsgPtr;  /**< Pointer for the introduction and interview process */

volatile bool FLAG_ARGB_CONFIG     = false;
volatile bool can_suspended        = false;
volatile bool can_driver_installed = false;
volatile bool FLAG_VALID_CONFIG    = false; /**< Clear the flag indicating a valid configuration  */

unsigned long previousMillis = 0;  /* will store last time a message was sent */
unsigned long lastCanError   = 0;  /* will store last time a CAN error occurred */

String wifiIP;

static const char *TAG = "canesp32";

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
/* Global strip pointers to allow for dynamic initialization */
// NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod>* strip = NULL;
// NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod>* g_argbStrips[MAX_SUB_MODULES] = { nullptr };
/**  store raw pointers because NeoPixelBus does not provide a polymorphic base class. */
void* g_argbStrips[MAX_SUB_MODULES] = { nullptr };

using RmtMethod0 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel0>;
using RmtMethod1 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel1>;
using RmtMethod2 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>;
using RmtMethod3 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>;
using RmtMethod4 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel4>;
using RmtMethod5 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel5>;
using RmtMethod6 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel6>;
using RmtMethod7 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel7>;

#endif

unsigned long ota_progress_millis = 0;

volatile bool wifi_connected = false;
volatile uint8_t myNodeID[NODE_ID_SIZE]; /**< node ID comprised of four bytes from MAC address */

/** Interrupt service routine for timer 0 */
void IRAM_ATTR Timer0_ISR()
{
  isrFlag = true;
}

static inline void setOutput(subModule_t& sub,
                             const personalityDef_t* p,
                             bool desiredState)
{
    gpio_num_t pin = (gpio_num_t)p->gpioPin;

    /* Apply logical inversion if required */
    bool electricalState = desiredState;
    if (p->capabilities & CAP_OUTPUT_INVERTED) {
        electricalState = !electricalState;
    }

    /* --------------------------------------------------------------------
     * Runtime reporting:
     * - Simple push‑pull GPIO → report 0/1 only
     * - Complex electrical modes → pack extended runtime bits
     * ------------------------------------------------------------------ */
    if (!(p->capabilities & (CAP_HIZ_OFF | CAP_HIZ_ON | CAP_OUTPUT_INVERTED)))
    {
        /* Simple push‑pull output: ON = 1, OFF = 0 */
        sub.runTime.valueU32 = electricalState ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
    }
    else
    {
        /* Extended runtime encoding for Hi‑Z, open‑drain, inverted logic */
        gpioExtendedRuntime_t rt = { .value = 0 };
        rt.bits.logicalState    = desiredState;
        rt.bits.electricalState = electricalState;
        rt.bits.hizOff          = (p->capabilities & CAP_HIZ_OFF)         ? 1U : 0U;
        rt.bits.hizOn           = (p->capabilities & CAP_HIZ_ON)          ? 1U : 0U;
        rt.bits.inverted        = (p->capabilities & CAP_OUTPUT_INVERTED) ? 1U : 0U;
        rt.bits.openDrain       = (p->capabilities & CAP_OPEN_DRAIN)      ? 1U : 0U;

        sub.runTime.valueU32 = rt.value;
    }

    /* --------------------------------------------------------------------
     * Electrical output behavior:
     * - If electricalState == true → ON behavior
     * - If electricalState == false → OFF behavior
     * - Hi‑Z flags override push‑pull drive
     * ------------------------------------------------------------------ */

    if (electricalState)
    {
        /* ON state: Hi‑Z ON (open‑drain release) */
        if (p->capabilities & CAP_HIZ_ON) {
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_INPUT);   /* float HIGH */
            return;
        }

        /* ON state: push‑pull HIGH */
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, GPIO_LEVEL_HIGH);
        return;
    }

    /* OFF state: Hi‑Z OFF (float LOW) */
    if (p->capabilities & CAP_HIZ_OFF) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);       /* float LOW */
    }
    else {
        /* OFF state: push‑pull LOW */
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, GPIO_LEVEL_LOW);
    }
}

static inline void subOutHelper(const uint8_t index, const bool state)
{
    if (index < MAX_SUB_MODULES)
    {
        subModule_t& sub = node.subModule[index];
        const personalityDef_t* p = &g_personalityTable[sub.personalityIndex];
        if (p)
        {
            setOutput(sub, p, state);
        }
    }
}

/** Inline function to validate sub-module index */
inline bool isValidSubModuleIndex(uint8_t index) 
{ 
  return (index < MAX_SUB_MODULES); 
}

/**
 * @brief Handles specialized FastLED initialization for ARGB sub-modules.
 * @param index The sub-module index (used to index ledData arrays).
 * @param sub   Reference to the sub-module configuration.
 */
void initArgbHardware(uint8_t index, subModule_t& sub)
{
#ifdef ARGB_LED

    if (index >= MAX_SUB_MODULES) return;

    const personalityDef_t* p = &g_personalityTable[sub.personalityIndex];
    if (!p) return;

    uint8_t  dataPin    = p->gpioPin;
    uint16_t pixelCount = sub.config.argb.ledCount;

    if (pixelCount > MAX_PIXEL_COUNT)
        pixelCount = MAX_PIXEL_COUNT;

    /* Free old strip if reinitializing */
    if (g_argbStrips[index] != nullptr)
    {
        free(g_argbStrips[index]);   // safe because we allocated with new
        g_argbStrips[index] = nullptr;
    }

    Serial.printf("ARGB using RMT channel %u\n", index);


    /* Instantiate strip with unique RMT channel */
    switch (index)
    {
        case 0:
            Serial.printf("Constructed type: RmtMethod%d\n", index);

            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod0>(pixelCount, dataPin);
            break;

        case 1:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod1>(pixelCount, dataPin);
            break;

        case 2:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod2>(pixelCount, dataPin);
            break;

        case 3:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod3>(pixelCount, dataPin);
            break;

        case 4:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod4>(pixelCount, dataPin);
            break;

        case 5:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod5>(pixelCount, dataPin);
            break;

        case 6:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod6>(pixelCount, dataPin);
            break;

        case 7:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod7>(pixelCount, dataPin);
            break;

        default:
            Serial.printf("ARGB Init ERROR: No RMT channel for submod %u\n", index);
            return;
    }

    /* Initialize strip */
    if (g_argbStrips[index] != nullptr)
    {
        // Cast back to the correct type based on index
        switch (index)
        {
            case 0:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[index])->Show();
                break;

            case 1:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[index])->Show();
                break;

            case 2:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[index])->Show();
                break;

            case 3:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[index])->Show();
                break;

            case 4:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[index])->Show();
                break;

            case 5:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[index])->Show();
                break;

            case 6:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[index])->Show();
                break;

            case 7:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[index])->Show();
                break;
        }

        Serial.printf("ARGB Init: submod %u pin %u count %u (RMT channel %u)\n",
                      index, dataPin, pixelCount, index);
    }

#endif
}



/**
 * @brief Initializes digital inputs such as physical switches and buttons.
 * @param i          The sub-module index (used to index submodule array).
 * @param sub        Reference to the sub-module configuration.
 * @details Configure the pull-up/pull-down resistor and initialize the input pin.
 * @note This function is called by the initHardware() function during setup().
 */
void initGPIOInput(uint8_t i, subModule_t& sub)
{
    const personalityDef_t* p = &g_personalityTable[sub.personalityIndex];  /**< Personality definition */
    const gpio_num_t pin = (gpio_num_t)p->gpioPin;                          /**< ESP‑IDF pin enum */
        
    gpio_config_t cfg = {};                                                 /**< GPIO config struct */
    cfg.mode         = GPIO_MODE_INPUT;                                     /**< Input mode */
    cfg.intr_type    = GPIO_INTR_DISABLE;                                   /**< ISR enabled later */
    cfg.pin_bit_mask = (1ULL << pin);                                       /**< Target pin mask */

    /* Configure pull resistors */
    const uint8_t pull = INPUT_FLAG_GET_PULL(sub.config.gpioInput.flags);
    switch (pull) {
        case INPUT_RES_PULLUP:
            cfg.pull_up_en   = GPIO_PULLUP_ENABLE;                  /**< Enable pull‑up */
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            Serial.printf("Submod %d: Digital Input Init Pull-Up (Pin %d)\n", i, p->gpioPin);
            break;

        case INPUT_RES_PULLDOWN:
            cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;                /**< Enable pull‑down */
            Serial.printf("Submod %d: Digital Input Init Pull-Down (Pin %d)\n", i, p->gpioPin);
            break;

        case INPUT_RES_FLOATING:
        default:
            cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;               /**< Floating input */
            Serial.printf("Submod %d: Digital Input Init Floating (Pin %d)\n", i, p->gpioPin);
            break;
    }

    gpio_config(&cfg);                                              /**< Apply configuration */
}


/**
 * @brief Initializes a PWM output sub-module.
 * @details This function is responsible for setting up the ESP32 LEDC driver for a PWM output sub-module.
 * The function takes two parameters: the index of the sub-module and a reference to the sub-module configuration.
 * The function sets up the LEDC driver with the channel set to the index, the frequency set to the configured frequency
 * multiplied by the PWM scaling factor, and the resolution set to 8-bit.
 * The function then attaches the PWM output pin to the LEDC driver and stores the pin number in the pwmPins array.
 * Finally, the function prints a debug message indicating the sub-module index, the PWM output pin, and the configured frequency.
 * @param i Index of the output switch / submodule.
 * @param sub Reference to the sub-module configuration.
 */
void initPwmHardware(uint8_t i, subModule_t& sub)
{
    const personalityDef_t* p = &g_personalityTable[sub.personalityIndex];   /**< Personality definition */
    const gpio_num_t pin = (gpio_num_t)p->gpioPin;                   /**< ESP‑IDF pin enum */

    /* ------------------------------------------------------------------------
     *  LEDC TIMER CONFIGURATION
     * ---------------------------------------------------------------------- */

    const uint32_t freq_hz =
        (uint32_t)(sub.config.gpioOutput.param1 * PWM_SCALING_FACTOR); /**< Scaled frequency */

    const ledc_timer_t timer =
        (ledc_timer_t)(i / LEDC_CHANNELS_PER_TIMER);                   /**< Timer index */

    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,                       /**< Low‑speed group */
        .duty_resolution  = LEDC_TIMER_8_BIT,                          /**< 8‑bit resolution */
        .timer_num        = timer,                                     /**< Timer selection */
        .freq_hz          = freq_hz,                                   /**< PWM frequency */
        .clk_cfg          = LEDC_USE_RTC8M_CLK                        /**< 8 MHz clock */
    };

    ledc_timer_config(&tcfg);                                          /**< Apply timer config */

    /* ------------------------------------------------------------------------
     *  LEDC CHANNEL CONFIGURATION
     * ---------------------------------------------------------------------- */

    const ledc_channel_t channel = (ledc_channel_t)i;                  /**< Channel index */

    ledc_channel_config_t ccfg = {
        .gpio_num       = pin,                                         /**< Output pin */
        .speed_mode     = LEDC_LOW_SPEED_MODE,                         /**< Low‑speed group */
        .channel        = channel,                                     /**< Channel selection */
        .intr_type      = LEDC_INTR_DISABLE,                           /**< No interrupts */
        .timer_sel      = timer,                                       /**< Timer binding */
        .duty           = 0,                                           /**< Start at 0% */
        .hpoint         = 0                                            /**< Default hpoint */
    };

    ledc_channel_config(&ccfg);                                        /**< Apply channel config */

    /* ------------------------------------------------------------------------
     *  BOOKKEEPING
     * ---------------------------------------------------------------------- */

    pwmPins[i] = pin;                                                  /**< Track pin assignment */

    Serial.printf("Submod %d: PWM Output Init (Pin %d, Freq %u Hz)\n",
                  i, p->gpioPin, freq_hz);
}


/**
 * @brief Clears a PWM pin's configuration and detaches it from the LEDC driver.
 * @details This function is typically called when a sub-module is deinitialized.
 * @param i Index of the output switch / submodule.
 */
void clearPwmHardware(uint8_t i) {
    ledcDetachPin(pwmPins[i]);
    pwmPins[i] = 0;
}

void initGpioOutput(uint8_t i, subModule_t& sub) {
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

  pinMode(p->gpioPin, OUTPUT);
  Serial.printf("Submod %d: Digital Output Init (Pin %d)\n", i, p->gpioPin);
} 

void initAnalogInput(uint8_t i, subModule_t& sub) {
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

  pinMode(p->gpioPin, ANALOG);
  Serial.printf("Submod %d: Analog Input Init (Pin %d)\n", i, p->gpioPin);
}

/**
 * @brief Initializes all sub-modules based on their intro message IDs.
 * @details Iterates through the sub-module array and calls the appropriate init function based on the intro message ID.
 * @note This function is called once by the main setup() function.
 */
void initHardware() {
  Serial.println("[HW] Initializing sub-modules...");

  for (int i = 0; i < node.subModCnt; i++) {
    subModule_t& sub = node.subModule[i];
    const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

    if (sub.introMsgId == 0) continue;

    switch (sub.introMsgId) {
      case DISP_ARGB_BACKLIGHT_ID:
      case DISP_ARGB_LED_STRIP_ID:
      case DISP_ARGB_BUTTON_BACKLIGHT_ID:
        initArgbHardware(i, sub);
        break;

      case INPUT_DIGITAL_GPIO_ID:
        initGPIOInput(i, sub);                           /* Configure the hardware */
        attachDigitalInputISR(p->gpioPin, i);            /* Register and attach the interrupt handler to the pin */
        if (sub.producer_flags & PRODUCER_FLAG_ENABLED){ /* If the producer is enabled */
          enableDigitalInputISR(p->gpioPin);             /* Enable the interrupt handler */
          Serial.printf("[HW] Submod %d: ISR Enabled (Pin %d)\n", i, p->gpioPin);
        }
        break;

      case OUT_GPIO_DIGITAL_ID:
      case DISP_ANALOG_BACKLIGHT_ID:
      case DISP_ANALOG_LED_STRIP_ID:
      case DISP_MONOCHROME_LED_ID:
      case DISP_STROBE_MODULE_ID:
        trackers[i].isConfigured = true; /* Mark as configured for the output task (blink, strobe and momentary) */
        initGpioOutput(i, sub);
        break;

      case OUT_GPIO_PWM_ID:
        initPwmHardware(i, sub);
        break;

      case INPUT_ANALOG_ADC_ID:
        initAnalogInput(i, sub);
        break;

      case DISP_TOUCHSCREEN_LCD_ID:
        /* Touchscreen init here if needed */
        Serial.printf("Submod %d: Touchscreen LCD Identified\n", i);
        break;

      default:
        /* No hardware init available */
        Serial.printf("Submod %d: No hardware init available for ID 0x%03X\n", i, sub.introMsgId);
        break;
    } /* switch (sub.introMsgId) */
  } /* for (int i = 0; i < MAX_SUB_MODULES; i++) */
} /* initHardware() */



/**
 * @brief Calculates a 16-bit CRC for the entire node configuration.
 */
const uint16_t getConfigurationCRC(const nodeInfo_t& node) {
  /* Hash the entire struct without worrying about internal fields */
  return crc16_ccitt((const uint8_t*)&node, sizeof(nodeInfo_t));
}

const uint16_t getSubModuleCRC(const subModule_t& sub) {
  /* Hash the submodule only */
  return crc16_ccitt((const uint8_t*)&sub, sizeof(subModule_t));
}

void TaskOTA(void *pvParameters) {
  /* Wait until WiFi is connected */
  while (!wifi_connected) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  Serial.println("[RTOS] OTA task started.");

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

    /* Suspend tasks that might be using hardware */
    if (xTWAIHandle != NULL)   vTaskSuspend(xTWAIHandle);   /* suspend the TWAI task */
    if (xOutputHandle != NULL) vTaskSuspend(xOutputHandle); /* suspend the output switch task */
    if (xInputHandle != NULL)  vTaskSuspend(xInputHandle);  /* suspend the input event task */
#ifdef ESP32CYD
    if (xDisplayHandle != NULL) vTaskSuspend(xDisplayHandle); /* suspend the display task */
    if (xTouchHandle != NULL) vTaskSuspend(xTouchHandle); /* suspend the touch task */
#endif
    /* Stop the TWAI driver */
    twai_stop();

    Serial.println("OTA Start: Background tasks suspended, starting flash... ");
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
  Serial.println("[OTA] OTA library ready");

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

  Serial.print("[INIT] Node ID extracted: ");
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
  Serial.println("[WIFI] STA Connected");
  Serial.print("[WIFI] STA SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("[WIFI] STA IPv4: ");
  Serial.println(WiFi.localIP());
  wifiIP = WiFi.localIP().toString();
}

/* when wifi disconnects */
void wifiOnDisconnect(){
  Serial.println("[WIFI] STA disconnected, reconnecting...");
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

extern "C" {
  /**
   * @brief Send a message to the CAN bus
   * @param msgid The message ID of the frame to be sent
   * @param data The data to be sent in the frame
   * @param dlc The data length code of the frame, which is the number of bytes of data to be sent
   */
  void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc) {
    twai_message_t message;
    static int failCount = 0; /* tx fail counter */

    #define CAN_STD_FRAME    0   /**< 0 = standard frame, 1 = extended frame */
    #define CAN_DATA_FRAME   0   /**< 0 = data frame, 1 = remote frame */
    #define CAN_NORMAL_TX    0   /**< 0 = normal transmission, 1 = SELF_RECEPTION 0 */
    #define CAN_NON_COMP_DLC 0   /**< non-compliant DLC (0-8 bytes) */
    
    if (dlc > CAN_MAX_DLC) dlc = CAN_MAX_DLC;    /* Safety check */

    /* Format message */
    message.identifier       = msgid;            /**< set message ID */
    message.extd             = CAN_STD_FRAME;    /**< 0 = standard frame, 1 = extended frame */
    message.rtr              = CAN_DATA_FRAME;   /**< 0 = data frame, 1 = remote frame */
    message.self             = CAN_NORMAL_TX;    /**< 0 = normal transmission, 1 = self reception request */
    message.dlc_non_comp     = CAN_NON_COMP_DLC; /**< non-compliant DLC (0-8 bytes) */
    message.data_length_code = dlc;              /**< data length code (0-8 bytes) */

    memcpy(message.data, data, dlc);             /**< copy data to message data field */

    const size_t dataSize = sizeof(message.data) / sizeof(message.data[0]);

    Serial.printf("[TWAI] Sending ID: 0x%03X DLC: %d DATA SIZE: %d\n", msgid, dlc, dataSize);

    /* Attempt transmission with a 10ms timeout */
    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
  #ifdef ESP32CYD
      digitalWrite(LED_RED, LOW); /* Turn on RED LED */
  #endif
      failCount = 0; /* Reset counter on successful queueing */
      // Serial.printf("ID: 0x%03X queued\n", msgid);
    } else {
      failCount++;
      Serial.printf("[TWAI] Tx Fail (%d/3)\n", failCount);

      if (failCount >= 3) {
        Serial.println("[TWAI] Persistent failure: Initiating TWAI Recovery...");

        /* Physical Bus Recovery Sequence */
        twai_stop();
        twai_initiate_recovery();

        vTaskDelay(pdMS_TO_TICKS(100)); /* Short delay for hardware state change */

        twai_start();
        Serial.println("[TWAI] TWAI Restarted");

        failCount = 0; /* Reset after recovery attempt */
      }
    }
    // vTaskDelay(10);
  #ifdef ESP32CYD
    digitalWrite(LED_RED, HIGH); /* Turn off RED LED */
  #endif

  } /* send_message() */
} /* extern "C" */

/** * @brief Configures LEDC hardware using the blinkerTracker_t structure.
 * @param idx  The index for both the blinker tracker and the LEDC timer (0-3).
 * @param pin  The hardware GPIO pin to use.
 * @param freq The desired frequency in Hertz (defaults to 5 Hz).
 */
void handleHardwarePwm(uint8_t submodIdx, uint8_t pin, uint32_t freq, uint32_t duty = (LEDC_13BIT_50PCT)) {
    uint8_t idx = (submodIdx - 1); /* submodule 0 will never be a blinker, so subtract 1 from index */

    /** * Safety check: Ensure index does not exceed hardware timer or array limits.
     */
    if (idx >= LEDC_MAX_TIMERS) {
        Serial.println("[PWM] Invalid index for LEDC timer, expected 0 to 3.");
        return;
    }

    /** * Cast the index to the appropriate LEDC types for the driver.
     * We map idx directly to Timer 0-3 and Channel 0-3.
     */
    ledc_timer_t selected_timer = static_cast<ledc_timer_t>(idx);
    ledc_channel_t selected_channel = static_cast<ledc_channel_t>(idx);

    /** * Update the blinkerTracker_t struct in the global array
     */
    blinkers[idx].freq     = freq;
    blinkers[idx].hwPin    = pin;
    blinkers[idx].subIdx   = submodIdx;

    /** * Configure the LEDC Timer
     */
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,     /* High resolution for low freq */
        .timer_num        = selected_timer,        /* Programmatic timer selection */
        .freq_hz          = freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /** * Configure the LEDC Channel
     */
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = selected_channel,        /* Unique channel for this output */
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = selected_timer,          /* Link channel to the specific timer */
        .duty           = duty,
        .hpoint         = 0,
        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    blinkers[idx].isActive = true;                 /* Set flag indicating blinker is active */

    // Serial.printf("Submod %d: Blinker Init (Pin %d) (%d Hz) (Duty %d) on Timer %d\n",
    //               submodIdx, pin, freq, duty, (int)selected_timer);
}

/**
 * @brief Stops an active LEDC blinker and unbinds the hardware pin.
 * @details Detaches the GPIO from the LEDC peripheral, resets the pin to LOW,
 * and clears the tracker status.
 * @param idx The index of the blinker in the global tracker array.
 */
void stopHardwarePwm(uint8_t submodIdx) {
    uint8_t idx = (submodIdx - 1); /* submodule 0 will never be a blinker, so subtract 1 from index */

    /** Safety check: Ensure index does not exceed hardware timer or array limits. */
    if (idx >= LEDC_MAX_TIMERS) {
        Serial.println("[PWM] Invalid index for LEDC stop timer, expected 0 to 3.");
        return;
    }


    /** Retrieve the pin from the tracker before clearing it in the array     */
    uint8_t pin = blinkers[idx].hwPin;
    if (blinkers[idx].isActive) {
        /** Detach the GPIO pin from the LEDC peripheral */
        ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(idx), 0); /* 0 sets the idle level to low */

        /** Explicitly set the pin to LOW to ensure a clean off-state  */
        // pinMode(pin, OUTPUT);
        // digitalWrite(pin, LOW);
        subOutHelper(submodIdx, false);

        /** Update tracker state */
        blinkers[idx].isActive = false;

        Serial.printf("[PWM] Submod %d: Blinker Stopped (Pin %d)\n", idx, pin);
    }
}

/**
 * @brief Handle momentary output behavior.
 *
 * Behavior:
 *   - When triggered, output goes HIGH
 *   - After duration expires, output returns LOW
 */
void handleMomentaryLogic(subModule_t &sub, outputTracker_t &trk) {
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

  uint8_t pin = p->gpioPin;

  if (!trk.isActive)
    return;

  /* If timer still running → keep output HIGH */
  if (millis() < trk.nextActionTime) {
    // digitalWrite(pin, HIGH);
    setOutput(sub, p, true);
    return;
  }

  /* Timer expired → turn output OFF */
  // digitalWrite(pin, LOW);
  setOutput(sub, p, false);
  trk.isActive = false;
}


void readCydLdr() {
#ifndef ESP32CYD
    return;
#else
    const uint8_t  oversample_count = 16;       /* Samples for precision */
    const uint8_t  ldr_submod_idx   = 5;        /* TODO: Make this configurable */
    
    uint32_t       raw_accumulator = 0;

    /* Perform over-sampling on pin 34 */
    Serial.printf("CYD LDR: %d\n", analogRead(CYD_LDR_PIN));
    for (uint8_t i = 0; i < oversample_count; i++) {
        raw_accumulator += analogRead(CYD_LDR_PIN);
    }
    
    /* Result is 12-bit (0-4095) */
    uint16_t ldr_raw = (uint16_t)(raw_accumulator / oversample_count);

    /* --- Goal 1: Send CAN Message (0x50F) --- */
    uint8_t data[DATA_ADC_RAW_DLC];
    /* Use your global myNodeID array */
    memcpy(&data[0], (const void*)myNodeID, 4);
    
    data[4] = ldr_submod_idx;           /* Sensor ID / Submodule index */
    data[5] = (uint8_t)(ldr_raw >> 8);  /* High byte of 12-bit ADC */
    data[6] = (uint8_t)(ldr_raw & 0xFF);/* Low byte of 12-bit ADC */

    /* Reusing your existing TWAI routine */
    send_message(DATA_ADC_RAW_ID, data, DATA_ADC_RAW_DLC);
#endif
}

static void setDisplayMode(twai_message_t& msg, uint8_t displayMode = DISPLAY_MODE_OFF) {
  uint8_t displayID = msg.data[4]; /* display ID */

  if (displayID >= MAX_SUB_MODULES) return; /* invalid display ID */
  subModule_t& sub = node.subModule[displayID]; /* get submodule reference */
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

  uint8_t displayPin = p->gpioPin; /* get output pin */

  switch (displayMode) {
    case DISPLAY_MODE_OFF: // display off
      // TODO: Add function to control backlight that handles multiple backlight types (analog, pwm, argb, etc)
      // stopHardwarePwm(displayID);
      // digitalWrite(displayPin, LOW);
      // setOutput(sub, p, false);
      Serial.printf("Display %d OFF\n", displayID);
      break;
    case DISPLAY_MODE_ON: // display on
      // stopHardwarePwm(displayID);
      // digitalWrite(displayPin, HIGH);
      // TODO: Add function to control backlight that handles multiple backlight types (analog, pwm, argb, etc)

      Serial.printf("Display %d ON\n", displayID);
      break;
    case DISPLAY_MODE_CLEAR: // clear display
      // stopHardwarePwm(displayID);
      // TODO: Not sure what "clear" means here
      Serial.printf("Display %d CLEAR\n", displayID);
      break;
    case DISPLAY_MODE_FLASH: // flash display
    {
      uint8_t flashRate = msg.data[5];
      // TODO: Need to support multiple backlight types
      handleHardwarePwm(displayID, displayPin, flashRate);
      Serial.printf("Display %d FLASH AT %d rate\n", displayID, flashRate);
    }
      break;
    default:
      Serial.println("Invalid display mode");
      break;
  }
}

static void setSwMomDur(twai_message_t& msg) {
  uint8_t switchID = msg.data[4];                  /* switch ID */
  uint8_t momDur = msg.data[5];                    /* momentary duration */
  if (switchID >= MAX_SUB_MODULES) return;         /* invalid switch ID */
  subModule_t& sub = node.subModule[switchID];     /* get submodule reference */

  sub.config.gpioOutput.param1 = momDur;   /* update momentary duration */

  Serial.printf("Momentary Duration: %d Switch: %d\n", sub.config.gpioOutput.param1, switchID);
}


static void setSwBlinkDelay(twai_message_t& msg) {
  uint8_t switchID = msg.data[4];                      /* switch ID */
  uint8_t freq     = msg.data[5];                      /* blink delay */
  if (switchID >= MAX_SUB_MODULES) return;             /* invalid switch ID */
  subModule_t& sub = node.subModule[switchID];         /* get submodule reference */
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */
  uint8_t pin = p->gpioPin; /* get output pin */
  sub.config.gpioOutput.param1 = freq;            /* update blink delay */
  handleHardwarePwm(switchID, pin, freq);       /* update hardware blinker */

  Serial.printf("Blink Delay: %d Switch: %d\n", sub.config.gpioOutput.param1, switchID);
}

static void setSwStrobePat(twai_message_t& msg) {
  uint8_t switchID = msg.data[4];               /* switch ID */
  uint8_t strobePat = msg.data[5];              /* strobe pattern */
  if (switchID >= MAX_SUB_MODULES) return;      /* invalid switch ID */
  subModule_t& sub = node.subModule[switchID];  /* get submodule reference */
  sub.config.gpioOutput.param2 = strobePat; /* update strobe pattern */

  Serial.printf("Strobe Pattern: %d Switch: %d\n", sub.config.gpioOutput.param2, switchID);
}


static void setPWMDuty(twai_message_t& msg) { /* 0x117 */
  uint8_t switchID = msg.data[4]; /* switch ID */
  double pwmDuty = (double)(msg.data[5] * 1.0) ;  /* pwm duty from master */
  pwmDuty = (double)(pwmDuty / 100.0); /* convert to decimal */
  pwmDuty = (double)(pwmDuty * LEDC_13BIT_100PCT); /* convert to LEDC duty cycle */
  if (switchID >= MAX_SUB_MODULES) return;      /* invalid switch ID */
  subModule_t& sub     = node.subModule[switchID];  /* get submodule reference */
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */
  uint8_t pin = p->gpioPin; /* get output pin */
  uint32_t workingFreq = (uint32_t)(sub.config.gpioOutput.param1 * PWM_SCALING_FACTOR);    /* get pwm frequency */
  handleHardwarePwm(switchID, pin, workingFreq, pwmDuty);     /* update hardware */
  Serial.printf("PWM Duty: %d Switch: %d\n", pwmDuty, switchID);
}

static void setPWMFreq(twai_message_t& msg) { /* 0x118 */
  uint8_t switchID = msg.data[4];  /* switch ID */
  uint8_t pwmFreq  = msg.data[5];  /* pwm frequency */
  if (switchID >= MAX_SUB_MODULES) return;      /* invalid switch ID */
  subModule_t& sub = node.subModule[switchID];  /* get submodule reference */
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */
  uint8_t pin = p->gpioPin; /* get output pin */
  sub.config.gpioOutput.param1 = pwmFreq;       /* update pwm frequency in config */
  uint32_t workingFreq = (uint32_t)(pwmFreq * PWM_SCALING_FACTOR);
  handleHardwarePwm(switchID, pin, workingFreq);     /* update hardware */
  Serial.printf("PWM Frequency: %d Switch: %d\n", workingFreq, switchID);
}

static void txSwitchState(uint8_t* txUnitID, uint16_t txSwitchID, uint8_t swState) {
  uint8_t dataBytes[8];
  static const uint8_t txDLC = 5;

  packUint32ToBytes(node.nodeID, dataBytes); /* pack node ID into buffer */
  dataBytes[4] = (txSwitchID); /* set switch ID */
  // dataBytes[5] = (swState); /* set switch state  */

  switch (swState) {

  case OUT_STATE_OFF: // switch off
    send_message(SW_SET_OFF_ID, dataBytes, SW_SET_OFF_DLC);
    break;
  case OUT_STATE_ON: // switch on
    send_message(SW_SET_ON_ID, dataBytes, SW_SET_ON_DLC);
    break;
  case OUT_STATE_MOMENTARY: // momentary press
    send_message(SW_MOM_PRESS_ID, dataBytes, SW_MOM_PRESS_DLC);
    break;
  default: // unsupported state
    Serial.println("Invalid switch state for transmission");
    break;
  }
}

/**
 * @brief Set the mode of a switch, does not set the state
 * @param msg The message containing the switch ID and mode
 *
 * This function takes a CAN message and sets the mode of the corresponding switch.
 * The switch can be set to one of the following modes:
 * - OUT_MODE_TOGGLE: Solid state (on/off)
 * - OUT_MODE_MOMENTARY: One-shot momentary
 * - OUT_MODE_STROBE: Strobe
 * - OUT_MODE_PWM: PWM
 * - OUT_MODE_BLINK: Same as PWM, but with slower timing
 *
 * If the switch is configured for momentary, blinking or pwm, the function will
 * set a flag for the outputTask to set the state of the switch accordingly.
 * Otherwise the function will set the state of the switch directly using the digitalWrite() function.
 *
 * @note This function will only work if the output is configured for digital output
 */
static void setSwitchMode(twai_message_t& msg) { /* 0x112 */
  uint8_t switchID = msg.data[4]; /* switch ID */
  uint8_t switchMode = msg.data[5];  /* switch mode */

  if (switchID >= MAX_SUB_MODULES) return; /* invalid switch ID */

  subModule_t& sub = node.subModule[switchID]; /* get submodule reference */

  clearPwmHardware(switchID); /* in case it was previously PWM */

  Serial.printf("Switch %d set to mode %d\n", switchID, switchMode);

  switch (switchMode) {
    case OUT_MODE_TOGGLE: // solid state (on/off)
      sub.config.gpioOutput.mode = switchMode;
      trackers[switchID].isActive = false;

      break;
    case OUT_MODE_MOMENTARY: // momentary
    case OUT_MODE_STROBE: // strobe 
      /** code handles both strobe and momentary modes */
      sub.config.gpioOutput.mode = switchMode;

      trackers[switchID].isConfigured = true;
      trackers[switchID].isActive = true;
      break;
    case OUT_MODE_BLINK:
    case OUT_MODE_PWM: // pwm and blinking
      sub.config.gpioOutput.mode = switchMode;

      trackers[switchID].isConfigured = false;
      trackers[switchID].isActive = false;

      // initPwmHardware(switchID, sub); /* call hardware setup helper*/
      break;
    default:
      Serial.println("Invalid switch mode");
      break;
  }

}

/**
 * @brief Set the state of a switch based on the configured mode
 * @param msg The message containing the switch ID and state
 * @param swState The state of the switch (OUT_STATE_OFF, OUT_STATE_ON, OUT_STATE_MOMENTARY)
 *
 * This function takes a CAN message and sets the state of the corresponding switch.
 * If the switch is configured for momentary, blinking or pwm, the function will
 * set a flag for the outputTask to set the state of the switch accordingly.
 * Otherwise the function will set the state of the switch
 * directly using the digitalWrite() function.
 *
 * @note This function will only work if the output is configured for digital output
 */
static void setSwitchState(twai_message_t& msg, uint8_t swState = OUT_STATE_OFF)
{ /* SET_SWITCH_STATE_ID */

  const uint8_t switchID = msg.data[4]; /* switch ID */

  if (switchID >= MAX_SUB_MODULES) return; /* invalid switch ID, exit function */

  subModule_t& sub = node.subModule[switchID]; /* get submodule reference */
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

  const uint8_t outPin = p->gpioPin;

  switch (swState) {
    case OUT_STATE_OFF: // switch off
      switch (sub.config.gpioOutput.mode) {
        case OUT_MODE_MOMENTARY:
          trackers[switchID].isActive = false;
          break;
        case OUT_MODE_STROBE:
          trackers[switchID].isActive = false;
          break;
        case OUT_MODE_BLINK:
        case OUT_MODE_PWM: // pwm and blinking
          stopHardwarePwm(switchID); 
          clearPwmHardware(switchID);
          break;
        default:
          // digitalWrite(outPin, LOW); /* set output driver low */
          setOutput(sub, p, false);
          Serial.printf("Switch %d (pin %d) OFF\n", switchID, outPin);
          break;
      }
      break;

    case OUT_STATE_ON: // switch on
      switch (sub.config.gpioOutput.mode) {
        case OUT_MODE_MOMENTARY: /* If we are in momentary mode, 'ON' should behave like a trigger */
          trackers[switchID].nextActionTime = millis() + 
            (MOM_SW_SCALING_FACTOR * sub.config.gpioOutput.param1);
          // digitalWrite(outPin, HIGH); /* Ensure it starts HIGH */
          setOutput(sub, p, true);
          trackers[switchID].isActive = true;
          break;
        case OUT_MODE_BLINK: /* If we are in blink mode, 'ON' should behave like a trigger */
        case OUT_MODE_PWM: /* Use LEDC hardware for blinking and pwm*/
        {
          //TODO: optimize this
          uint32_t freq = (uint32_t)(sub.config.gpioOutput.param1 * BLINK_SCALING_FACTOR);
          uint8_t  pin  = outPin;
          if (freq == 0) { /* for debugging don't let the blink rate equal 0*/
            freq = 5; //TODO: FIX THIS NO MAGIC NUMBERS
            sub.config.gpioOutput.param1 = freq;
          }
          handleHardwarePwm(switchID, pin, freq);
          break;
        }

        default:
          // digitalWrite(outPin, HIGH); /* set output driver high */
          setOutput(sub, p, true);
          Serial.printf("Switch %d (pin %d) ON\n", switchID, outPin);
          break;
      }
      break;

    case OUT_STATE_MOMENTARY: // momentary press
      /* let the output task deal with this */
      Serial.printf("Output Task Switch %d (pin %d) MOMENTARY (%dms)\n", switchID, outPin, (MOM_SW_SCALING_FACTOR * sub.config.gpioOutput.param1));

      trackers[switchID].nextActionTime = millis() + (MOM_SW_SCALING_FACTOR * sub.config.gpioOutput.param1);
      // digitalWrite(outPin, HIGH); /* Ensure it starts HIGH */
      setOutput(sub, p, true);
      trackers[switchID].isActive = true; /* enable momentary timer */
      trackers[switchID].isConfigured = true; /* enable this output for nonblocking control*/
      break;

    default:
      Serial.println("Invalid switch state");
      break;
  }
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
 * @brief Execute one step of the strobe pattern for an output-capable submodule.
 *
 * This function:
 *   - Uses a pattern table (durations in ms)
 *   - Advances through the pattern when the timer expires
 *   - Sets the output HIGH/LOW based on the pattern step
 *   - Updates tracker.nextActionTime for the next step
 *
 * The pattern is defined by the submodule configuration (strobePattern_t).
 * Each pattern is an array of ON/OFF durations.
 */
void handleStrobeLogic(subModule_t &sub, outputTracker_t &trk) {
  const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */
  const uint8_t outPin = p->gpioPin;

  /* If the strobe is not active, do nothing */
  if (!trk.isActive)
    return;

  /* If it's not time to advance the pattern, do nothing */
  if (millis() < trk.nextActionTime)
      return;

  /* Select the pattern based on user configuration */
  uint8_t patternId = sub.config.gpioOutput.param2;

  if (patternId >= (sizeof(STROBE_PATTERNS) / sizeof(STROBE_PATTERNS[0]))) /* Invalid pattern, turn off output and exit */
  {
      trk.isActive = false;
      // digitalWrite(outPin, LOW);
      setOutput(sub, p, false);
      return;
  }

  /** Get a pointer to the pattern */
  const StrobePatternDef& def = STROBE_PATTERNS[patternId];

  if (!def.steps || def.count == 0) /* Invalid pattern, turn off output and exit */
  {
      trk.isActive = false;
      // digitalWrite(outPin, LOW);
      setOutput(sub, p, false);
      return;
  }

  /* Load the pattern specifics */
  const uint16_t* pattern = def.steps;
  uint8_t    patternSteps = def.count;

  /* Advance to the next step in the pattern */
  trk.currentStep = (trk.currentStep + 1) % patternSteps;

  /* Even steps = ON, Odd steps = OFF */
  bool state = (trk.currentStep % 2 == 0);

  /* Set the GPIO output state */
  // digitalWrite(outPin, state);
  setOutput(sub, p, state);

  /* Schedule the next step */
  trk.nextActionTime = millis() + pattern[trk.currentStep];

  /* Record the GPIO state in runTime */
  sub.runTime.valueU32 = packStrobeState(
    sub.config.gpioOutput.param2,   // pattern ID
    trk.currentStep,                // step index
    state                           // GPIO state
  );

}


/**
 * @brief Handles color commands from the gateway node.
 *
 * @param msg The TWAI message containing the color command.
 * 
 */
void handleColorCommand(twai_message_t& msg)
{
#ifdef ARGB_LED
    uint8_t subIdx     = msg.data[4];
    uint8_t colorIndex = msg.data[5];

    if (subIdx >= MAX_SUB_MODULES) return;
    if (g_argbStrips[subIdx] == nullptr) return;
    if (colorIndex >= COLOR_PALETTE_SIZE) return;

    subModule_t& sub = node.subModule[subIdx];
    PaletteColor p = SystemPalette[colorIndex];

    /* Update the color value in runTime */
    sub.runTime.valueU32       = packRgb(p.R, p.G, p.B);
    sub.runTime.last_change_ms = millis();

    switch (subIdx)
    {
        case 0:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 1:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 2:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 3:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 4:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 5:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 6:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 7:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[subIdx])
                ->Show();
            break;
    }

    Serial.printf("ARGB[%d] = (%d,%d,%d)\n", subIdx, p.R, p.G, p.B);
#endif
}


void manageColorPickerList(twai_message_t& msg) {
#ifndef ESP32CYD  
  return; /* exit function unless we are running on CYD board */
#else
    /* Constants for NVS and Byte logic to avoid magic numbers */
    const char* NVS_NAMESPACE = "cyd_nodes";
    const char* NVS_KEY       = "node_list";

    uint32_t cmd = msg.identifier;

    /* Extract Node ID from data payload if the message contains one */
    uint32_t targetNodeId = 0;
    if (msg.data_length_code >= CAN_NODE_ID_LEN) 
    {
        targetNodeId = ((uint32_t)msg.data[0] << 24) | 
                       ((uint32_t)msg.data[1] << 16) | 
                       ((uint32_t)msg.data[2] << 8)  | 
                        (uint32_t)msg.data[3];
    }

    switch (cmd) 
    {
        case COLORPICKER_READ_NVS_ID: 
        {
            Preferences prefs;
            prefs.begin(NVS_NAMESPACE, true); /* Read-only mode */
            
            size_t bytesAvailable = prefs.getBytesLength(NVS_KEY);
            
            /* Calculate capacity of our existing array in bytes */
            size_t maxArrayBytes = sizeof(ARGBNode) * MAX_ARGB_NODES;

            if (bytesAvailable > 0 && bytesAvailable <= maxArrayBytes) 
            {
                prefs.getBytes(NVS_KEY, (void*)discoveredNodes, bytesAvailable);
                discoveredNodeCount = bytesAvailable / sizeof(ARGBNode);
                Serial.printf("ARGB: Loaded %d nodes from NVS\n", discoveredNodeCount);
            }
            prefs.end();
            break;
        }

        case COLORPICKER_WRITE_NVS_ID: 
        {
            Preferences prefs;
            prefs.begin(NVS_NAMESPACE, false); /* Read-write mode */
            
            /* Save only the active portion of the array */
            size_t bytesToWrite = sizeof(ARGBNode) * discoveredNodeCount;
            prefs.putBytes(NVS_KEY, (const void*)discoveredNodes, bytesToWrite);
            
            prefs.end();
            Serial.println("ARGB: Node list persisted to NVS");
            break;
        }

        case COLORPICKER_PURGE_LIST_ID: 
        {
            memset((void*)discoveredNodes, 0, sizeof(discoveredNodes));
            discoveredNodeCount = 0;
            Serial.println("ARGB: List purged from memory");
            break;
        }

        case COLORPICKER_ADD_ROUTE_ID: 
        {
            if (targetNodeId == 0) return;

            /* Check for existing entry to prevent duplicates */
            bool alreadyExists = false;
            for (int i = 0; i < discoveredNodeCount; i++) 
            {
                if (discoveredNodes[i].id == targetNodeId) 
                {
                    alreadyExists = true;
                    break;
                }
            }

            if (!alreadyExists && (discoveredNodeCount < MAX_ARGB_NODES)) 
            {
                discoveredNodes[discoveredNodeCount].id = targetNodeId;
                discoveredNodes[discoveredNodeCount].lastColorIdx = 0; /* Default to Black/Off */
                discoveredNodes[discoveredNodeCount].active = true;
                discoveredNodeCount++;
                Serial.printf("ARGB: Node 0x%08X added to picker\n", targetNodeId);
            }
            break;
        }

        case COLORPICKER_DEL_ROUTE_ID: 
        {
            for (int i = 0; i < discoveredNodeCount; i++) 
            {
                if (discoveredNodes[i].id == targetNodeId) 
                {
                    /* Shift remaining nodes to maintain a contiguous list */
                    for (int j = i; j < (discoveredNodeCount - 1); j++) 
                    {
                        discoveredNodes[j] = discoveredNodes[j + 1];
                    }
                    discoveredNodeCount--;
                    Serial.printf("ARGB: Node 0x%08X removed\n", targetNodeId);
                    break;
                }
            }
            break;
        }

        case COLORPICKER_SEND_LIST_ID: 
        {
            /* Not yet implemented */
            /* Respond to master with current list status */
            // twai_message_t response;
            // response.identifier = COLORPICKER_SEND_LIST_ID;
            // response.data_length_code = 1;
            // response.data[0] = (uint8_t)discoveredNodeCount;
            // twai_transmit(&response, pdMS_TO_TICKS(10));
            break;
        }

        default:
            Serial.printf("ARGB: Unknown picker management command: 0x%03X\n", cmd);
            break;
    }
#endif
} /* end manageColorPickerList() */



/**
 * @brief Dynamically counts active submodules in the node structure
 * * Scans the subModule array and counts entries with a non-zero introMsgId.
 * This eliminates the need to hard-code subModCnt in the default config.
 * * @return uint8_t Total number of configured submodules
 */
uint8_t countActiveSubModules() {
    uint8_t count = 0;
    /* Iterate through the maximum possible submodules (8) */
    for (int i = 0; i < MAX_SUB_MODULES; i++) {
        if (node.subModule[i].introMsgId != 0) {
            count++;
        }
    }
    return count;
}

/* send the router table to the master node */
void sendRouteList()
{
  uint8_t       msgData[CAN_MAX_DLC] = {0};           /* wipe the buffer before using it */
  uint16_t      txMsgID              = 0;             /* Init to 0 */
  uint32_t      txMsgDLC             = CAN_MAX_DLC;   /* Init to 8 bytes */
  const uint8_t msgDataOffset        = 5;             /* skip the four bytes for node id and one byte for chunk id */
  uint8_t       routeCount           = 0;             /* Init to 0 */
  uint16_t      crc                  = 0xFFFF;        /* Initial value */

  // Count active routes
  for (int i = 0; i < MAX_ROUTES; i++) {
    if (g_routesCrc[i].in_use)
        routeCount++;
  }

  // Compute total chunks
  const uint8_t chunksPerRoute = ROUTE_CHUNKS_PER_ROUTE;
  const uint8_t totalChunks = routeCount * chunksPerRoute;

  /* compute route table crc16 */
  crc = crc16_ccitt((const uint8_t*)g_routes, sizeof(g_routes));

  /* Consistent 32-bit Node ID across all route frames */
  packUint32ToBytes(node.nodeID, &msgData[MSG_DATA_0]);

  /* Step 1: Send route list header */
  txMsgID  = ROUTE_LIST_BEGIN_ID;
  txMsgDLC = ROUTE_LIST_BEGIN_DLC;

  msgData[MSG_DATA_4] = routeCount;  /* set route count */
  msgData[MSG_DATA_5] = totalChunks; /* set total chunks */

  /* Send the message */
  send_message(txMsgID, msgData, txMsgDLC);

  /* Step 2: Send route list data */
  txMsgID  = ROUTE_LIST_DATA_ID;
  txMsgDLC = ROUTE_LIST_DATA_DLC;

  /* Reset the buffer, reload the node id */
  memset(msgData, 0, CAN_MAX_DLC);
  packUint32ToBytes(node.nodeID, &msgData[MSG_DATA_0]);

  uint8_t chunkIdx = 0;
  for (int idx = 0; idx < MAX_ROUTES; idx++) {
    if (g_routesCrc[idx].in_use) {

      /* Update CRC value with the CRC for this route entry */
      crc = crc16_ccitt_update(crc,
                                 (const uint8_t*)&g_routes[idx],
                                 sizeof(route_entry_t));

      /* Copy the route entry to the message buffer, 3 bytes at a time */
      const uint8_t *raw = (const uint8_t*)&g_routes[idx]; /* pointer to route data*/

      for (uint8_t chunk = 0; chunk < chunksPerRoute; chunk++) {
        msgData[MSG_DATA_4] = chunkIdx;         /* chunk index */
        msgData[MSG_DATA_5] = raw[chunk*3 + 0]; /* payload byte 0 */
        msgData[MSG_DATA_6] = raw[chunk*3 + 1]; /* payload byte 1 */
        msgData[MSG_DATA_7] = raw[chunk*3 + 2]; /* payload byte 2 */

        /* Send the message */
        send_message(txMsgID, msgData, txMsgDLC);

        chunkIdx++; /* increment chunk index */
      }
    }
  }

  /* Step 3: send end of route list message, include total routes, total chunks and crc16 */
  txMsgID  = ROUTE_LIST_END_ID;
  txMsgDLC = ROUTE_LIST_END_DLC;

  /* Reset the buffer, reload the node id */
  memset(msgData, 0, CAN_MAX_DLC);
  packUint32ToBytes(node.nodeID, &msgData[MSG_DATA_0]);

  msgData[MSG_DATA_4] = routeCount;           /* total route count      */
  msgData[MSG_DATA_5] = chunkIdx;             /* counter of chunks sent */
  msgData[MSG_DATA_6] = ((crc >> 8) & 0xFF);  /* set crc16 high byte    */
  msgData[MSG_DATA_7] = (crc & 0xFF);         /* set crc16 low byte     */
  
  /* Send the message */
  send_message(txMsgID, msgData, txMsgDLC);

} /* end sendRouteList() */

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
  uint8_t  msgData[CAN_MAX_DLC];


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
    // Serial.printf("TX INTRO: NODE 0x%08X SUBMOD %02u (Type: 0x%03X, CRC: 0x%04X)\n", node.nodeID, msgData[4], txMsgID, txCrc);
  }
/* >0: Sub-module Identity (Part A and Part B) */
  else {
    uint8_t modIdx = (uint8_t)((msgPtr - 1) / 2); /**< Map ptr to sub-module index */
    bool isPartB   = ((msgPtr - 1) % 2) != 0;      /**< Alternate A/B sequence */
    
    if (modIdx >= node.subModCnt) return;
    subModule_t& sub = node.subModule[modIdx];
    const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

    const uint16_t dataMsgId  = p->dataMsgId;
    const uint8_t  dataMsgDlc = p->dataMsgDlc; 

    txMsgID = sub.introMsgId;

    if (!isPartB) {
        /* Part A: Configuration Data */
        msgData[4] = modIdx; /**< bits 0-6: index, bit 7: 0 (Part A) */
        msgData[5] = sub.config.rawConfig[0];
        msgData[6] = sub.config.rawConfig[1];
        msgData[7] = sub.config.rawConfig[2];
    } else {
        /* Part B: Telemetry/Operational Data */
        msgData[4] = modIdx | SUBMOD_PART_B_FLAG; /**< Set bit 7 to indicate Part B */
        msgData[5] = (uint8_t)(dataMsgId >> 8);
        msgData[6] = (uint8_t)(dataMsgId);
        /* Pack DLC (4 bits) and SaveState (1 bit) into byte 7 */
        msgData[7] = (dataMsgDlc & 0x0F) | (sub.submod_flags ? SUBMOD_PART_B_FLAG : 0x00);
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

static void buildSyntheticMessage(const router_action_t &action,
                                  twai_message_t &outMsg)
{
    // Start with a clean message
    memset(&outMsg, 0, sizeof(outMsg));

    outMsg.identifier = action.actionMsgId;
    if (action.actionMsgDlc > 0) {
      outMsg.data_length_code = action.actionMsgDlc;
    } else {
      outMsg.data_length_code = CAN_MAX_DLC;   // Always 8 for consumer actions
    }

    // Bytes 0–3 = NodeID (same as incoming messages)
    packUint32ToBytes(node.nodeID, &outMsg.data[0]);

    // Byte 4 = submodule index
    outMsg.data[4] = action.sub_idx;

    // Byte 5..7 = parameters from router
    outMsg.data[5] = action.param[0];
    outMsg.data[6] = action.param[1];
    outMsg.data[7] = action.param[2];
}


static void handleCanRX(twai_message_t &message) {
  // twai_message_t altmessage;
  bool msgFlag = false;

  if ((message.identifier == SET_ARGB_STRIP_COLOR_ID) || (message.identifier == SET_ARGB_BUTTON_COLOR_ID)) {
    handleColorCommand(message);
  }

  if (message.data_length_code > 0) { // message contains data, check if it is for us
    uint8_t rxUnitID[4] = {message.data[0], message.data[1], message.data[2], message.data[3]};
    // memcmp((const uint8_t *)rxUnitID, (const uint8_t *)myNodeID, 4);

    if (memcmp(message.data, (const uint8_t *)myNodeID, 4) == 0) {
      msgFlag = true; // message is for us
    } else {
      msgFlag = false; // message is not for us
    }
  } else {
    msgFlag = true; // general broadcast message is valid, messagae has no node id assigned
    Serial.printf("RX BROADCAST MSG: 0x%x NO DATA\n", message.identifier);
  }

  if (!msgFlag) {
    // Serial.println("Message does not match our ID, end of process.");
    return; // message is not for us
  }

  /* prepare router library action buffer */
  router_action_t action = {0};
  /* prepare synthetic message buffer */
  twai_message_t msgToConsume = {0};

  /* recast message as can_msg_t so it matches the checkRoutes() signature */
  const can_msg_t newMsg = toCanMsg(&message);

  /* hand off message to the router library */
  bool takeAction = checkRoutes(&newMsg, &action);

  /* decide if we generate a synthetic message or use the original */
  if (takeAction) { /* message router indicates we need to generate a synthetic message */
      Serial.printf("[ROUTER] ACTION: 0x%03X\n", action.actionMsgId);
      buildSyntheticMessage(action, msgToConsume);
  } else {
      msgToConsume = message;  // use original
      /* debug: dump message data */
      Serial.printf("[ROUTER] RX MSG: 0x%03X DATA: ", message.identifier);
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.printf("0x%02X ", message.data[i]);
      }
      Serial.println();
  }

  /** extract submodule index */
  uint8_t modIdx = msgToConsume.data[4];                                                     /* byte 4 holds the sub module index */


  switch (msgToConsume.identifier) 
  {
    case ROUTE_TAKE_NO_ACTION:          // no action 0xFFFF
      break;

    case DATA_ROUTE_ACK_ID:
      send_message(DATA_ROUTE_ACK_ID, msgToConsume.data, DATA_ROUTE_ACK_DLC);
      break;

    case REQ_ROUTE_LIST_ID:
      sendRouteList();
      break;

    case CFG_PRODUCER_CFG_ID: /**< Configure a single producer submodule */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      handleProducerCfg(&msgToConsume);
      break;

    case CFG_PRODUCER_WRITE_NVS_ID: /**< Commit producer config to NVS. */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      requestProducerSave();
      break;

    case CFG_PRODUCER_READ_NVS_ID: /**< Request producer config for all submodules */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      requestProducerLoad();
      break;

    case REQ_PRODUCER_CFG_ID: /**< Request producer config  for idx */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;

    case RESP_PRODUCER_CFG_ID: /**< Node responds with requested data using this message id */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;

    case CFG_PRODUCER_PURGE_ID: /**< Purge the producer list */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
        producerPurgeSingle(modIdx);
      }
      break;

    case CFG_PRODUCER_DEFAULTS_ID: /**< Reset the producer at idx to defaults */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      producerDefaultSingle(modIdx);
      break;

    case CFG_PRODUCER_APPLY_ID: /**< NO-OP Producer config is applied instantly */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;


    case CFG_PRODUCER_ENABLE_ID: /**< Enable the producer at idx */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        const personalityDef_t* p = &g_personalityTable[node.subModule[modIdx].personalityIndex];
        if (!p) 
        {
          Serial.printf("Personality not found for index %d\n", modIdx);
          break;
        }
        producerEnable(modIdx);            /* Enable the producer */
        enableDigitalInputISR(p->gpioPin); /* Enable the digital input interrupt */

        break;
      }

    case CFG_PRODUCER_DISABLE_ID: /**< Disable the producer at idx */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        const personalityDef_t* p = &g_personalityTable[node.subModule[modIdx].personalityIndex];
        if (!p) 
        {
          Serial.printf("Personality not found for index %d\n", modIdx);
          break;
        }
        producerDisable(modIdx);            /* Disable the producer */
        disableDigitalInputISR(p->gpioPin); /* Disable the digital input interrupt */
        break;
      }

    case CFG_PRODUCER_TOGGLE_ID:        /**< Toggle operation of producer at idx */
      // Not implemented
      break;

    case REQ_PRODUCER_LIST_ID: /**< Ask the node to dump the entire producer cfg list */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;
    

    case PRODUCER_LIST_BEGIN_ID: /**< Node will announce the count of defined producers */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;

    case PRODUCER_LIST_DATA_ID: /**< Producer cfg data for index */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;

    case PRODUCER_LIST_END_ID: /**< Node will announce the end of the defined producers list */
      if (isValidSubModuleIndex(modIdx) == false) {
        Serial.printf("Invalid sub module index %d\n", modIdx);
        break;
      }
      break;

    case SW_SET_PWM_DUTY_ID:      // set output switch pwm duty
      setPWMDuty(msgToConsume);
      break;
    case SW_SET_PWM_FREQ_ID:      // set output switch pwm frequency
      setPWMFreq(msgToConsume);
      break;
    case SW_SET_MODE_ID:           // setup output switch modes
      setSwitchMode(msgToConsume);
      break;
    case SW_SET_OFF_ID:            // set output switch off
      setSwitchState(msgToConsume, OUT_STATE_OFF);
      break;
    case SW_SET_ON_ID:             // set output switch on
      setSwitchState(msgToConsume, OUT_STATE_ON);
      break;
    case SW_MOM_PRESS_ID:          // set output switch momentary press
      setSwitchState(msgToConsume, OUT_STATE_MOMENTARY);
      break;
    case SW_SET_BLINK_DELAY_ID:          // set output switch blink delay
      setSwBlinkDelay(msgToConsume);
      break;
    case SW_SET_STROBE_PAT_ID:          // set output switch strobe pattern
      setSwStrobePat(msgToConsume);
      break;
    case SET_DISPLAY_OFF_ID:          // set display off
      setDisplayMode(msgToConsume, DISPLAY_MODE_OFF);
      break;
    case SET_DISPLAY_ON_ID:          // set display on
      setDisplayMode(msgToConsume, DISPLAY_MODE_ON);
      break;
    case SET_DISPLAY_FLASH_ID:          // flash display backlight
      setDisplayMode(msgToConsume, DISPLAY_MODE_FLASH);
      break;
    case SET_ARGB_STRIP_COLOR_ID:          /* set ARGB color */
      handleColorCommand(msgToConsume); /* byte 4 is the sub module index, byte 5 is the color index */
      break;
    case CFG_SUB_DATA_MSG_ID:           /* setup sub module data message */
      /* no longer user configured */
      break;
    case CFG_SUB_INTRO_MSG_ID:          /* setup sub module intro message */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.introMsgId    = ((msgToConsume.data[5] << 8) | (msgToConsume.data[6] & 0xFF));    /* bytes 5:6 hold the intro message ID */
        sub.introMsgDLC   = msgToConsume.data[7];                               /* byte 7 holds the intro message DLC */
        Serial.printf("Update Sub %d INTRO MSG: 0x%03X DLC: %d\n", modIdx, sub.introMsgId, sub.introMsgDLC);
      }

      break;

    case CFG_ARGB_STRIP_ID:                                                     /* setup ARGB channel */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];
        sub.config.argb.reserved   = msgToConsume.data[5];       /* reserved, not used */
        sub.config.argb.ledCount   = msgToConsume.data[6];       /* byte 6 holds the number of LEDs (max 255)*/
        sub.config.argb.colorOrder = msgToConsume.data[7];       /* byte 7 holds the color order */
      }
      break;
    case CFG_DIGITAL_INPUT_ID: /**< Setup digital input channel */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.config.gpioInput.flags       = msgToConsume.data[5];  /* Input resistor configuration */
        sub.config.gpioInput.debounce_ms = msgToConsume.data[6];  /* input debounce time */
        sub.config.gpioInput.reserved    = msgToConsume.data[6];  /* reserved byte */

      }
      break;

    case CFG_ANALOG_INPUT_ID: /**< Setup analog ADC input channel */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.config.analogInput.overSampleFlag = msgToConsume.data[5];
        sub.config.analogInput.reserved1      = msgToConsume.data[6]; /* store reserved byte if sent */
        sub.config.analogInput.reserved2      = msgToConsume.data[7]; /* store reserved byte if sent */
      }
      break;

    case CFG_BLINK_OUTPUT_ID: /**< Setup blinking/strobing output channel */
    case CFG_PWM_OUTPUT_ID: /**< Setup PWM output channel */
    case CFG_DIGITAL_OUTPUT_ID: /**< Setup digital output channel (relays/mosfets) */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.config.gpioOutput.mode   = msgToConsume.data[5]; /* output mode, see defines */
        sub.config.gpioOutput.param1 = msgToConsume.data[6]; /* paramater byte 0 (varies) */
        sub.config.gpioOutput.param2 = msgToConsume.data[7]; /* parameter byte 1 */
      }
      break;

    case CFG_ANALOG_STRIP_ID: /**< Setup analog RGB/RGBW strip */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.config.analogStrip.configIndex = msgToConsume.data[5];
        sub.config.analogStrip.reserved1   = msgToConsume.data[6];
        sub.config.analogStrip.reserved2   = msgToConsume.data[7];
      }
      break;

    case CFG_ANALOG_OUTPUT_ID: /**< Setup analog DAC output channel */
      {
        if (isValidSubModuleIndex(modIdx) == false) {
          Serial.printf("Invalid sub module index %d\n", modIdx);
          break;
        }
        subModule_t& sub = node.subModule[modIdx];

        sub.config.analogOutput.outputMode  = msgToConsume.data[5];
        sub.config.analogOutput.param1      = msgToConsume.data[6];
        sub.config.analogOutput.param2      = msgToConsume.data[7];

        /** msgToConsume.data[7] is reserved/padding */
      }
      break;
    case CFG_ERASE_NVS_ID: /**< 0x41B: Master requesting NVS erase */
      {
        Serial.println("Master requesting NVS erase...");
        handleEraseCfgNVS();
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

        uint16_t masterCrc = (msgToConsume.data[4] << 8) | msgToConsume.data[5];
        uint16_t localCrc  = getConfigurationCRC(node);

        uint8_t responseData[6];
        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(node.nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8);
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        /** CRCs match, attempt to persist to flash */
        if (saveConfigNvs(node) == CFG_OK) {
            /** SUCCESS: Set the flag indicating config is valid and send back the verified CRC */
            FLAG_VALID_CONFIG = true;
            send_message(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
            Serial.println("NVS Commit Successful");
        } else {
            /** Flash hardware error */
            send_message(DATA_CFGWRITE_FAILED_ID, responseData, DATA_CFGWRITE_FAILED_DLC);
            Serial.println("NVS Commit Failed: Flash Error");
        }
      }
      break;
    case CFG_READ_NVS_ID: /**< 0x41E: Master requesting NVS read */
      {
        Serial.println("NVS Read Request");
        uint16_t masterCrc = (message.data[4] << 8) | message.data[5];

        handleReadCfgNVS();

        uint16_t localCrc  = getConfigurationCRC(node);
        uint8_t responseData[6];

        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(node.nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8);
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        send_message(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
      }
      break;
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
      {
      // Use explicit casting to prevent shift overflow
      uint32_t epochTime;
      epochTime = ((uint32_t)msgToConsume.data[4] << 24) |
                  ((uint32_t)msgToConsume.data[5] << 16) |
                  ((uint32_t)msgToConsume.data[6] << 8)  |
                   (uint32_t)msgToConsume.data[7];
      setEpochTime((uint32_t)epochTime);
      Serial.println("Received epoch from master; updating clock");
      }
      break;

      case COLORPICKER_ADD_ROUTE_ID: /* 0x433: colorpicker add node */
      case COLORPICKER_DEL_ROUTE_ID: /* 0x432: colorpicker del node */
      case COLORPICKER_PURGE_LIST_ID: /* 0x431: colorpicker purge list */
      case COLORPICKER_SEND_LIST_ID: /* 0x430: colorpicker send list */
      case COLORPICKER_WRITE_NVS_ID: /* 0x42F: colorpicker write nvs */
      case COLORPICKER_READ_NVS_ID: /* 0x42E: colorpicker read nvs */
        manageColorPickerList(msgToConsume);
      break;

    default:
      Serial.printf("Unknown message received 0x%x\n", msgToConsume.identifier);
      // sendIntroack();
      break;
  }
} // end of void handleCanRX()

void updateSubModules()
{
  for (uint8_t i = 0; i < node.subModCnt; i++)
  {
    subModule_t          *sub = &node.subModule[i];
    const personalityDef_t *p = &g_personalityTable[sub->personalityIndex];

    if (!p) {
        Serial.printf("Producer: personality lookup failed for index %u\n", i);
        return;
    }

    /* Lookup the pin */
    uint8_t myPin = p->gpioPin;

    /* generic storage for value */
    uint32_t value = 0;

    /* skip disabled submodules */
    if (sub->submod_flags & SUBMOD_FLAG_DISABLED)
      continue;

    switch (sub->personalityId)
    {
      /* Addressable RGB output*/
      case PERS_ARGB_OUTPUT:
        // TODO: Figure out how to read NeoPixelBus status?
        break;

      /* Analog RGBW output */
      case PERS_RGBW_OUTPUT:
        // TODO
        break;

      /* Analog DAC output */
      case PERS_ANA_OUTPUT:
        // TODO: Read DAC output value?
        break;
      
      /* Digital GPIO personalities */
      case PERS_GPIO_OUTPUT:
      case PERS_GPIO_INPUT:
        // sub->runTime.valueU32 = digitalRead(myPin); /**< Capture digital value */
        break;

        /* Analog ADC input */
      case PERS_ANALOG_INPUT:
        sub->runTime.valueU32 = analogRead(myPin); /**< Capture analog value */
        break;

      case VIRT_FREE_HEAP:
          value = (uint32_t)xPortGetFreeHeapSize();
          break;

      case VIRT_WIFI_RSSI:
      {
        int rssi = WiFi.RSSI();
        if (rssi < 0) rssi = -rssi;
        value = (uint32_t)rssi;
        break;              
      }

      case VIRT_RTOS_HIGHWATERMARK:
        value = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
        break;

      case VIRT_INTERNAL_TEMPERATURE:
        value = (uint32_t)temperatureRead();
        break;

      case VIRT_VREF_VOLTAGE:
        // value = (uint32_t)readInternalVref();
        value = 1024; // TODO: implement actual function
        break;

      default:
        // Unknown virtual personality — ignore safely
        continue;
      }

      // sub->runTime.valueU32       = value;     /* update value */
      // sub->runTime.last_change_ms = millis();  /* update timestamp */
    } /* end of for loop */
}


static void handleProducerTick(uint32_t now)
{
    /** Check for producer event */
    producer_event_t evt = producerTick(now); 

    if (evt.error) {
        Serial.println("[ERR] ProducerTick returned error, aborting producer processing");
        return;
    }
    if (!evt.ready)
        return;

    if (evt.sub_idx >= node.subModCnt) {
        Serial.printf("[ERR] ProducerTick: bad sub_idx %u (max %u)\n",
                      evt.sub_idx, node.subModCnt - 1);
        return;
    }        

    /** Lookup personality */
    const subModule_t    &sub = node.subModule[evt.sub_idx];
    const personalityDef_t *p = &g_personalityTable[sub.personalityIndex];
    if (!p) {
        Serial.printf("Producer: personality lookup failed for sub %u\n", evt.sub_idx);
        return;
    }

    uint8_t dlc = p->dataMsgDlc;
    if (dlc < CAN_DATAMSG_MIN_DLC) dlc = CAN_DATAMSG_MIN_DLC;  /**< enforce minimum DLC for producer messages 6-bytes */
    if (dlc > CAN_MAX_DLC) dlc = CAN_MAX_DLC; /**< enforce maximum DLC */

    uint8_t payload[CAN_MAX_DLC] = {0}; /**< zero-initialize */

    /* Node ID (big-endian) */
    payload[0] = (node.nodeID >> BYTE_SHIFT3) & BYTE_MASK;
    payload[1] = (node.nodeID >> BYTE_SHIFT2) & BYTE_MASK;
    payload[2] = (node.nodeID >> BYTE_SHIFT)  & BYTE_MASK;
    payload[3] = (node.nodeID)                & BYTE_MASK;

    /* Submodule index */
    payload[4] = evt.sub_idx;

    const uint32_t valueU32 = evt.value;

    if (valueU32 <= 0xFF) {                 /* Single byte value, send it in position 5 */
        payload[5] = valueU32 & BYTE_MASK;        
    } else if (valueU32 <= 0xFFFF) {        /* Two-byte value */
        payload[5] = (valueU32 >> BYTE_SHIFT) & BYTE_MASK;
        payload[6] = (valueU32)               & BYTE_MASK;

    } else {                                /* Three-byte value */
        payload[5] = (valueU32 >> BYTE_SHIFT2) & BYTE_MASK;
        payload[6] = (valueU32 >> BYTE_SHIFT)  & BYTE_MASK;
        payload[7] = (valueU32)                & BYTE_MASK;
    }

    /* Send message */
    send_message(p->dataMsgId, payload, dlc);

    Serial.printf("ProducerTx: sub %u msg 0x%03X dlc %u val %u\n",
                  evt.sub_idx, p->dataMsgId, dlc, evt.value);
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
        readCydLdr(); /* Read CYD LDR and send that data to the bus */

        /* If we are stuck mid-sequence, reset after 10s of silence */
        if (introMsgPtr != 0) {
            Serial.println("Intro sequence timed out, resetting pointer.");
            introMsgPtr = 0;
        }
    }

  /** Update physical submodules */
  updateSubModules();

  /** Call the producer tick */
  handleProducerTick(currentMillis);
    
}

/**
 * @brief Process a GPIO event and convert it into value for the producer
 *
 * @param subIdx The submodule index of the GPIO event
 * @param raw The raw value of the GPIO event (0 or 1)
 *
 * This function processes a GPIO event and converts it into a producer subsystem.
 * It first checks if the submodule index is valid and if the pin is enabled.
 * Then it applies the inversion, debouncing and stable state detection according to the mode of the GPIO event.
 * Finally, it updates the runTime struct of the submodule.
 */
static void processGpioEvent(uint8_t subIdx, uint8_t raw)
{
  if (subIdx < 0 || subIdx >= MAX_SUB_MODULES)
      return;

  subModule_t* sub          = &node.subModule[subIdx]; /**< pointer to the submodule */
  const personalityDef_t* p = &g_personalityTable[sub->personalityIndex]; /**< pointer to the personality definition */
  const uint8_t pinNum      = p->gpioPin; /**< GPIO pin number */

  /** Test for invalid pin number */
  if (pinNum >= GPIO_NUM_MAX)
    return;

  /** Test if ISR is enabled for this pin */
  if (!isrGpio.enabled[pinNum])
    return;

  Serial.printf("[INPUT] sub=%u pin=%u raw=%u\n",
            subIdx, pinNum, raw);

  const gpio_num_t pin         = (gpio_num_t)pinNum; /**< GPIO pin */
  const uint32_t   now         = millis();  /**< timestamp in ms */

  const uint8_t    debounceMs  = sub->config.gpioInput.debounce_ms;
  const uint8_t    flags       = sub->config.gpioInput.flags;

  const inputModeType_t mode   = (inputModeType_t)INPUT_FLAG_GET_MODE(flags);
  const inputInvert_t   inv    = (inputInvert_t)INPUT_FLAG_GET_INV(flags);


  /* Apply inversion */
  const uint8_t value = inv ? !raw : raw; /**< Invert raw value if requested, store as output value */

  /* Raw change detection */
  if (value != isrGpio.lastRawState[pinNum]) {
      isrGpio.lastRawState[pinNum] = value;
      isrGpio.lastChangeMs[pinNum] = now;
  }
  // Serial.printf("[RAW] pin=%u value=%u lastRaw=%u lastChange=%u\n",
  //             pinNum,
  //             value,
  //             isrGpio.lastRawState[pinNum],
  //             isrGpio.lastChangeMs[pinNum]);

              

  // Serial.printf("[DEBOUNCE] now=%u lastChange=%u diff=%u debounceMs=%u\n",
  //             now,
  //             isrGpio.lastChangeMs[pinNum],
  //             now - isrGpio.lastChangeMs[pinNum],
  //             debounceMs);

  /* Debounce window, skip it if debounce is disabled */
  if (debounceMs != INPUT_DEBOUNCE_DISABLED && 
      ((now - isrGpio.lastChangeMs[pinNum]) < debounceMs))
      return;

  /* Stable state detection */
  if (value == isrGpio.stableState[pinNum]) {
      return;  // no stable change, nothing to do
  }

  isrGpio.stableState[pinNum]  = value;
  isrGpio.lastStableMs[pinNum] = now;

  // Serial.printf("[STABLE] sub=%u value=%u mode=%u now=%u\n",
  //             subIdx, value, mode, now);



  /* ============================
    *  MODE: MOMENTARY
    * ============================ */
  if (mode == INPUT_MODE_MOMENTARY) {
      sub->runTime.valueU32       = value ? MOMENTARY_ACTIVE_VALUE
                                          : MOMENTARY_RELEASE_VALUE;
      sub->runTime.last_change_ms = now;
      // Serial.printf("[MOMENTARY] sub=%u valueU32=%u\n",
      //         subIdx, sub->runTime.valueU32);
  }
  /* ============================
    *  MODE: TOGGLE
    * ============================ */
  else if (mode == INPUT_MODE_TOGGLE && value == GPIO_STATE_HIGH) {
      sub->runTime.valueU32      ^= TOGGLE_BIT_MASK;   // toggle bit 0
      sub->runTime.last_change_ms = now;
      // Serial.printf("[TOGGLE] sub=%u valueU32=%u\n",
      //         subIdx, sub->runTime.valueU32);
  }
  /* ============================
    *  MODE: LATCH
    * ============================ */
  else if (mode == INPUT_MODE_LATCH) {
      sub->runTime.valueU32       = value ? GPIO_LATCH_ON : GPIO_LATCH_OFF;
      sub->runTime.last_change_ms = now;
      // Serial.printf("[LATCH] sub=%u valueU32=%u\n",
      //         subIdx, sub->runTime.valueU32);
  }
  /* ============================
    *  MODE: NORMAL BUTTON
    *  (click, double-click, long press)
    * ============================ */
  else if (mode == INPUT_MODE_NORMAL) {

      if (value == GPIO_STATE_HIGH) {
          isrGpio.pressStartMs[pinNum] = now;
      } else {
          uint32_t pressDuration = now - isrGpio.pressStartMs[pinNum];

          if (pressDuration >= NORMAL_LONG_PRESS_MS) {
              sub->runTime.valueU32 = GPIO_LONG_PRESS;   // long press
          } else {
              if ((now - isrGpio.lastClickMs[pinNum]) < NORMAL_DOUBLE_CLICK_MS) {
                  sub->runTime.valueU32      = GPIO_DOUBLE_CLICK;   // double click
                  isrGpio.clickCount[pinNum] = 0;
              } else {
                  sub->runTime.valueU32      = GPIO_SINGLE_CLICK;   // single click
                  isrGpio.clickCount[pinNum] = 1;
              }
              isrGpio.lastClickMs[pinNum] = now;
          }

          sub->runTime.last_change_ms = now;
      }
    // Serial.printf("[NORMAL] sub=%u valueU32=%u\n",
    //         subIdx, sub->runTime.valueU32);
  } else
  {
    /* fall-through: if mode doesn't match, we just leave runTime unchanged */
    Serial.printf("[UNKNOWN] sub=%u valueU32=%u\n",
          subIdx, sub->runTime.valueU32);
  } /* end of mode switch */
}


void TaskTWAI(void *pvParameters) {
  // give some time at boot for the cpu setup other parameters
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("[RTOS] TWAI task started.");

  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); /* accept all messages, filter in software */

  /* Install TWAI driver */
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("[TWAI] TWAI installed.");
  } else {
    Serial.println("[TWAI] Failed to install TWAI.");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Start TWAI driver */
  if (twai_start() == ESP_OK) {
    Serial.println("[TWAI] TWAI started.");
  } else {
    Serial.println("[TWAI] Failed to start TWAI, reboot recommended.");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states */
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("[TWAI] TWAI alerts reconfigured.");
  } else {
    Serial.println("[TWAI] Failed to reconfigure alerts.");
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
        Serial.println("[TWAI] Alert: Controller has become error passive.");
        lastErrPassLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      if (millis() - lastBusErrLog > LOG_INTERVAL) {
        Serial.printf("[TWAI] Alert: Bus error. Count: %d\n", twaistatus.bus_error_count);
        lastBusErrLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      if (millis() - lastTxFailLog > LOG_INTERVAL) {
        Serial.println("[TWAI] Alert: Transmission failed.");
        lastTxFailLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      if (millis() - lastRxFullLog > LOG_INTERVAL) {
        Serial.println("[TWAI] Alert: RX queue full.");
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
        handleCanRX(message); /* Process the message */
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

/**
 * @brief Periodic task that manages all output-capable submodules.
 *
 * This task:
 *   - Iterates over all submodules
 *   - Skips anything that is not an output personality
 *   - Applies the behavior defined by outputMode_t
 *   - Uses trackers[] for timing-based modes (momentary, strobe)
 *
 * Runs every 10 ms → 100 Hz timing resolution.
 */
void TaskOutput(void *pvParameters)
{
    Serial.println("[RTOS] Output task running... ");

    for (;;)
    {
        for (int i = 0; i < MAX_SUB_MODULES; i++)
        {
          /** Reference to the current sub-module */
          subModule_t &sub = node.subModule[i];

          /** Pointer to the personality definition for this sub-module */
          const personalityDef_t* p = &g_personalityTable[sub.personalityIndex]; 

          /** Reference to the output tracker for this sub-module */
          outputTracker_t &trk = trackers[i];

          /* Skip submodules that are not configured for output behavior */
          if (!trk.isConfigured)
            continue;

          /* Skip personalities that do not support output behavior */
          if (!(p->capabilities & CAP_OUTPUT))
            continue;

          /* Dispatch based on the configured output mode */
          switch (sub.config.gpioOutput.mode) {
            case OUT_MODE_MOMENTARY:
              handleMomentaryLogic(sub, trk);
              break;

            case OUT_MODE_STROBE:
              handleStrobeLogic(sub, trk);
              break;
              
            case OUT_MODE_BLINK:
            case OUT_MODE_PWM:
              /** 
               * BLINK and PWM are handled in a separate helper,
               * nothing to do here
               */
              break;

            case OUT_MODE_ALWAYS_ON:
              if (!trk.hasBeenSet) {
                // digitalWrite(p->gpioPin, HIGH);
                setOutput(sub, p, true);
                trk.hasBeenSet = true;
              }
              break;

            case OUT_MODE_ALWAYS_OFF:
              if (!trk.hasBeenSet) {
                // digitalWrite(p->gpioPin, LOW);
                setOutput(sub, p, false);
                trk.hasBeenSet = true;
              }
              break;

            case OUT_MODE_FOLLOW:
              /* FOLLOW mode reacts only to incoming CAN events,
                  so the output task does nothing here. */
              break;

            case OUT_MODE_TOGGLE:
              /* TOGGLE is also event-driven; nothing to do here. */
              break;

            default:
              /* Unknown mode — do nothing */
              break;
          } /* switch (sub.config.digitalOutput.outputMode) */
        } /* for (int i = 0; i < MAX_SUB_MODULES; i++) */

        /* 10 ms timing resolution */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void TaskInput(void *pvParameters) {
  Serial.println("[RTOS] GPIO event task started\n");
  gpio_event_t evt;

    for (;;) {
      /* Handle GPIO events */
      if (xQueueReceive(gpioEventQueue, &evt, portMAX_DELAY)) {
        processGpioEvent(evt.subIdx, evt.raw);
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    
}

void setup() {
  /** clear config valid flag */ 
  FLAG_VALID_CONFIG    = false;

  adc_power_acquire();

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
  delay(2500); /* Provide time for the CPU to settle after reboot */

  /* Debug check for memory alignment */
  Serial.printf("\n[DEBUG] Struct Sizes - nodeInfo_t: %d, subModule_t: %d\n",
              sizeof(nodeInfo_t), sizeof(subModule_t));

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  /* set up some clock parameters */
  setenv("TZ", DEFAULT_TIMEZONE, ENV_VAL_OVERWRITE);
  tzset();

  readMacAddress(); /**< Read the ESP32 station MAC address and program myNodeID */

  /* Initialize memory */
  Serial.println("[INIT] Initializing memory...");
  memset(&node, 0, sizeof(nodeInfo_t));                            /* Clear nodeInfo struct */
  memset(&trackers, 0, sizeof(outputTracker_t) * MAX_SUB_MODULES); /* clear outputTracker struct */
  memset(&pwmPins, 0, sizeof(uint8_t) * LEDC_MAX_TIMERS);          /* clear the array of pwm pins */

  /* Load nodeID into the nodeInfo struct */
  node.nodeID = unpackBytestoUint32((const uint8_t*)&myNodeID);

  /* Read the NVS data from flash */
  handleReadCfgNVS();                                                 

  /* Install ISR service */
  initGpioIsrService();  

  /* Initialize the hardware, attach ISR handlers as needed */
  initHardware();

  /** Initialize producer library callbacks */
  producerInit(&producerCB);

  /* Initialize router CRC16 callback */
  router_set_crc_callback(crc16_ccitt);

  #ifdef ESP32CYD
  initCYD();                                                       /* Initialize CYD interface */
  analogSetAttenuation(ADC_11db);
  pinMode(CYD_LDR, INPUT); /* Setup the LDR as an input TODO: this should be a submodule */
  #endif

  /* Start the CAN task */
  xTaskCreate(
    TaskTWAI,              /* Task function */
    "Task TWAI",           /* name of task */
    TASK_TWAI_STACK_SIZE,  /* Stack size of task */
    NULL,                  /* parameter of the task */
    tskNormalPriority,     /* priority of the task */
    &xTWAIHandle           /* Task handle to keep track of created task */
  );

  /* Start OTA task  */
  xTaskCreate(
    TaskOTA,
    "Task OTA",
    TASK_OTA_STACK_SIZE,
    NULL,
    tskHighPriority,
    NULL
  );

  /* Start the output handler task*/
  xTaskCreate(
    TaskOutput,
    "Task Output Switch",
    TASK_OUTPUT_STACK_SIZE,
    NULL,
    tskLowPriority, /* lowest priority plus one */
    &xOutputHandle
  );

  /* Start the input event task*/
  xTaskCreate(
    TaskInput,
    "Task Input Events",
    TASK_INPUT_STACK_SIZE, /* 4096 bytes */
    NULL,
    tskNormalPriority, /* normal priority */
    &xInputHandle
  );
}

void printWifi() {
  Serial.println("");
  Serial.print("[INIT] Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  /** Check for producer save request */
  if (g_routeSaveRequested) {
    g_routeSaveRequested = false;
    saveRoutesToNVS();
  }

  /** Check for producer load request */
  if (g_routeLoadRequested) {
    g_routeLoadRequested = false;
    loadRoutesFromNVS();
  }

  /** Check for submodule save request */
  for (int i=0; i < MAX_SUB_MODULES; i++) {
    if (node.subModule[i].submod_flags & SUBMOD_FLAG_DIRTY) {
      /** Save the submodule */
      const ConfigStatus sts = saveSubModuleNvs(node.subModule[i], i);
      if (sts == CFG_OK) {
        /** Clear the dirty flag after successful save */
        node.subModule[i].submod_flags &= ~SUBMOD_FLAG_DIRTY;
      }
    }
  }

  if (g_routeSaveRequested) { /* Check for router save request */
    g_routeSaveRequested = false;
    saveRoutesToNVS();
  }

  if (g_routeLoadRequested) { /* Check for router load request */
    g_routeLoadRequested = false;
    loadRoutesFromNVS();
  }
  
  vTaskDelay(20 / portTICK_PERIOD_MS);

  // NOP;
}