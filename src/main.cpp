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
// #include <ArduinoOTA.h>

/* === ESP32 includes === */
// #include <Preferences.h>
// #include <rom/crc.h>
// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <esp_wifi.h>
#include <time.h>

/* === Standard library includes === */
#include <stdio.h>
#include <stddef.h>

/* === ESP32 native IDF includes === */
// #include "driver/twai.h" /* esp32 native TWAI CAN library */
// #include "driver/ledc.h" /* esp32 native LEDC PWM library */
// #include "driver/gpio.h" /* esp32 native GPIO library */
#include "driver/adc.h" /* esp32 native ADC library */
#include "esp_err.h"    /* esp32 error handler */
#include "soc/gpio_struct.h"

/* === FreeRTOS includes === */
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

/* === Local includes === */
// #include "argb_hw.h"           /**< ARGB LED control */
// #include "can_dispatch.h"      /**< CAN dispatch logic */
// #include "can_types.h"         /**< CAN data type definitions */
// #include "colorpalette.h"      /**< Color palette definitions */
// #include "freertos.h"          /**< FreeRTOS definitions */
// #include "node_state.h"        /**< Node and sub-module state table */
// #include "secrets.h"           /**< WiFi credentials and such */

#include "hardware_init.h" /**< Hardware initialization routines */
#include "isr_gpio.h"      /**< GPIO interrupt routines */
#include "pwm_hw.h"        /**< PWM LEDC routines */
#include "storage.h"       /**< NVS storage routines */
#include "task_consumer.h" /**< CAN consumer logic */
#include "task_input.h"    /**< Task input routines */
#include "task_ota.h"      /**< Task OTA routi6nes */
#include "task_output.h"   /**< Task output routines */
#include "task_producer.h" /**< Task producer routines */
#include "task_twai.h"     /**< Task TWAI routines */
#include "timers.h"        /**< Timer functions */
#include "wifi_hw.h"       /**< WiFi initialization and control */

/* external includes */
#include "canbus_project.h" /**< my various CAN functions and structs */
#include "can_router.h"     /**< CAN routing (subcriber) routines and constants */
#include "can_producer.h"   /**< CAN producer routines and constants */
#include "can_platform.h"   /**< Platform-specific routines and constants */

#include "personality_table.h" /**< For initRuntimePersonalityTable */
// #include "submodule_types.h"   /**< Sub-module type definitions */
// #include "strobe_patterns.h"   /**< Strobe pattern definitions from can_personality */
// #include "byte_conversion.h"   /**< Byte conversion functions */

/* hardware constants */
// #include "esp32_defs.h"        /**< ESP32 hardware definitions for personality table */

#include "esp_log.h"
extern "C" uint32_t isrGetCounter(void);

static const char *TAG = "main";

#ifdef ESP32CYD
#include "espcyd.h"
#endif

// #define CAN_ID_MASK            (0x3F)      /**< Mask for lower 6 bits of CAN ID */
// #define PWM_RES_BITS           (8U)        /**< 8-bit resolution for PWM */

/* Real time clock macros */
#define DEFAULT_TIMEZONE "EST5EDT,M3.2.0,M11.1.0"
#define ENV_VAL_OVERWRITE (1U)

/* Default CAN transceiver pins */
// #ifndef RX_PIN
// #define RX_PIN 22
// #endif

// #ifndef TX_PIN
// #define TX_PIN 21
// #endif

// #ifndef NODEMSGID
// #define NODEMSGID 0x701
// #endif

/* CAN bus polling intervals */
// #define TRANSMIT_RATE_MS (100U)
// #define POLLING_RATE_MS  (100U)

/* WiFi Constants */
// #define AP_SSID  "canesp32"

/* LEDC constants */
// #define LEDC_CHANNELS_PER_TIMER (8U)   /**< ESP32: 8 channels per timer */

/* Connect node state functions to producer library callback table */
static const producerCallbacks_t producerCB = {
    .getSubModuleCount = nodeGetActiveSubModuleCount,
    .getSubModule = nodeGetSubModule,
    .getRuntime = nodeGetRuntime};

/* definitions from external libraries */
#ifdef ESP32CYD
// TODO: move these to espcyd.h
// extern int discoveredNodeCount; /**< Track active count in the array */
// extern const int MAX_ARGB_NODES;
// extern ARGBNode discoveredNodes[];
#endif

/* ESP-IDF wrapper for ESP_LOG - supports logging from external libraries */
// TODO: Not supported by ESP32-Arduino
//  extern "C" void myPlatformLogger(int level, const char *fmt, va_list args)
//  {
//      switch (level) {
//          case 0: ESP_LOGE("LIB", fmt, args); break;
//          case 1: ESP_LOGW("LIB", fmt, args); break;
//          case 2: ESP_LOGI("LIB", fmt, args); break;
//          default: ESP_LOGD("LIB", fmt, args); break;
//      }
//  }

/**
 * @brief Append template personalities referenced by submodules.
 *
 * Hardware + internal personalities are already present in the runtime table.
 * This routine scans all submodules (after NVS load) and ensures that any
 * template-based personalityId referenced by a submodule is present in the
 * runtime personality table.
 *
 * If the personalityId already exists in the runtime table, the submodule's
 * personalityIndex is corrected (in case NVS stored a stale index).
 *
 * If the personalityId does not exist, it is appended from templateTable[].
 *
 * @return int
 *         >0 : number of template personalities appended
 *          0 : nothing appended (success)
 *         -1 : failure (missing template, table full, invalid state)
 */
static int appendTemplatePersonalities(void)
{
    int appended = 0;

    const uint8_t subModCnt = nodeGetActiveSubModuleCount();

    /* Iterate over all submodules currently defined */
    for (uint8_t s = 0; s < subModCnt; s++)
    {

        /* STEP 1: Validation */

        /* Skip pre-defined submodules */
        if (s < g_personalityCount)
        {
            ESP_LOGI(TAG, "[INIT] Skipping pre-defined submodule at index %d", s);
            continue;
        }

        /* Get a pointer to the current submodule */
        subModule_t *sub = nodeGetActiveSubModule(s);

        /* Get the current personalityId */
        uint16_t pid = sub->personalityId;

        /* ------------------------------------------------------------
         * Step 2: PersonalityId not found in runtime table.
         *         Look for it in the templateTable[].
         * ------------------------------------------------------------ */
        const personalityDef_t *src = NULL;

        for (uint8_t i = 0; i < g_TemplateCount; i++)
        {
            if (templateTable[i].personalityId == pid)
            {
                /* Attempt to locate the template in the templateTable[] */
                src = &templateTable[i];
                break;
            }
        }

        if (src == NULL)
        {
            /* Template personality missing — fatal configuration error */
            ESP_LOGW(TAG,
                     "[ERR] appendTemplatePersonalities(): personalityId 0x%03X not found in templateTable",
                     pid);
            return -1;
        }

        /* ------------------------------------------------------------
         * Step 3: Ensure runtime table has space for another personality.
         * ------------------------------------------------------------ */
        if (runtimePersonalityCount >= MAX_RUNTIME_PERSONALITIES)
        {
            ESP_LOGW(TAG, "[ERR] appendTemplatePersonalities(): runtime table full");
            return -1;
        }

        /* ------------------------------------------------------------
         * Step 4: Append the template personality to the runtime table.
         * ------------------------------------------------------------ */
        /* Find a free index in the personality table */
        const int freeSlot = getFreePersonalitySlot();

        if (freeSlot < 0)
        {
            ESP_LOGW(TAG, "[ERR] appendTemplatePersonalities(): runtimePersonalityTable is full");
            return -1; /* early exit with error */
        }

        /* We have a free index slot, use it as the index */
        const uint8_t newIndex = (uint8_t)freeSlot;

        /* Copy the template to the runtime table */
        runtimePersonalityTable[newIndex] = *src;

        /* Update submodule to reference the new runtime index */
        sub->personalityIndex = newIndex;

        /* Mark the submodule as dirty for NVS commit */
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;

        ESP_LOGI(TAG,
                 "[INIT] Appended template personality 0x%03X at runtime index %d",
                 pid, newIndex);

        /* Increment appended count */
        appended++;
    }

    return appended; /* Success */
}

static void manageColorPickerList(can_msg_t *msg)
{
#ifndef ESP32CYD
    return; /* exit function unless we are running on CYD board */
#else
    /* Constants for NVS and Byte logic to avoid magic numbers */
    const char *NVS_NAMESPACE = "cyd_nodes";
    const char *NVS_KEY = "node_list";

    uint32_t cmd = msg->identifier;

    /* Extract Node ID from data payload if the message contains one */
    uint32_t targetNodeId = 0;
    if (msg->data_length_code >= CAN_NODE_ID_LEN)
    {
        targetNodeId = ((uint32_t)msg->data[0] << 24) |
                       ((uint32_t)msg->data[1] << 16) |
                       ((uint32_t)msg->data[2] << 8) |
                       (uint32_t)msg->data[3];
    }

    switch (cmd)
    {
    case COLORPICKER_READ_NVS_ID:
    {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, true); /* Read-only mode */

        size_t bytesAvailable = prefs.getBytesLength(NVS_KEY);

        /* Calculate capacity of our existing array in bytes */
        size_t maxArrayBytes = sizeof(argbNode_t) * MAX_ARGB_NODES;

        if (bytesAvailable > 0 && bytesAvailable <= maxArrayBytes)
        {
            prefs.getBytes(NVS_KEY, (void *)discoveredNodes, bytesAvailable);
            discoveredNodeCount = bytesAvailable / sizeof(argbNode_t);
            ESP_LOGI(TAG, "ARGB: Loaded %d nodes from NVS", discoveredNodeCount);
        }
        prefs.end();
        break;
    }

    case COLORPICKER_WRITE_NVS_ID:
    {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, false); /* Read-write mode */

        /* Save only the active portion of the array */
        size_t bytesToWrite = sizeof(argbNode_t) * discoveredNodeCount;
        prefs.putBytes(NVS_KEY, (const void *)discoveredNodes, bytesToWrite);

        prefs.end();
        ESP_LOGI(TAG, "ARGB: Node list persisted to NVS");
        break;
    }

    case COLORPICKER_PURGE_LIST_ID:
    {
        memset((void *)discoveredNodes, 0, sizeof(discoveredNodes));
        discoveredNodeCount = 0;
        ESP_LOGI(TAG, "ARGB: List purged from memory");
        break;
    }

        // case COLORPICKER_ADD_ROUTE_ID:
        // {
        //     if (targetNodeId == 0) return;

        //     /* Check for existing entry to prevent duplicates */
        //     bool alreadyExists = false;
        //     for (int i = 0; i < discoveredNodeCount; i++)
        //     {
        //         if (discoveredNodes[i].id == targetNodeId)
        //         {
        //             alreadyExists = true;
        //             break;
        //         }
        //     }

        //     if (!alreadyExists && (discoveredNodeCount < MAX_ARGB_NODES))
        //     {
        //         discoveredNodes[discoveredNodeCount].id = targetNodeId;
        //         discoveredNodes[discoveredNodeCount].lastColorIdx = 0; /* Default to Black/Off */
        //         discoveredNodes[discoveredNodeCount].active = true;
        //         discoveredNodeCount++;
        //         ESP_LOGI(TAG, "ARGB: Node 0x%08X added to picker", targetNodeId);
        //     }
        //     break;
        // }

        // case COLORPICKER_DEL_ROUTE_ID:
        // {
        //     for (int i = 0; i < discoveredNodeCount; i++)
        //     {
        //         if (discoveredNodes[i].id == targetNodeId)
        //         {
        //             /* Shift remaining nodes to maintain a contiguous list */
        //             for (int j = i; j < (discoveredNodeCount - 1); j++)
        //             {
        //                 discoveredNodes[j] = discoveredNodes[j + 1];
        //             }
        //             discoveredNodeCount--;
        //             ESP_LOGI(TAG, "ARGB: Node 0x%08X removed", targetNodeId);
        //             break;
        //         }
        //     }
        //     break;
        // }

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
        ESP_LOGW(TAG, "ARGB: Unknown picker management command: 0x%03X", cmd);
        break;
    }
#endif
} /* end manageColorPickerList() */

void setup()
{

    /*
     * Keep ADC powered on all the time to prevent a weird
     * glitch with the WiFI subsystem and GPIO pin 39
     */
    adc_power_acquire();

#ifdef ESP32CYD
    /* register callbacks */
    // TODO: Update ESPCYD library to use new twai and output_task interface
    //   espcyd_set_send_message_callback(canEnqueueMessage);
    //   espcyd_set_backlight_callback(handleHardwarePwm);
#endif

#ifndef BAUD
#define BAUD 115200
#endif

    /* setup serial port */
    Serial.begin(BAUD);
    Serial.setDebugOutput(true);

    delay(2500);
    wifiEnable(); /* start the WiFi task */

    /* set up some clock parameters */
    setenv("TZ", DEFAULT_TIMEZONE, ENV_VAL_OVERWRITE);
    tzset();

    /* Initialize memory */
    ESP_LOGI(TAG, "[INIT] Initializing memory...");
    nodeInit();           /* Initialize the node state array */
    pwmHwInit();          /* Initialize LEDC memory resources */
    freeRtosInit();       /* Initialize FreeRTOS resources */
    nodeReadMacAddress(); /* Read the ESP32 station MAC address and program node.nodeID */

    /* Initialize runtime personality table (built-ins only) */
    int rc = initRuntimePersonalityTable();
    if (rc > 0)
    {
        runtimePersonalityCount = rc;
        ESP_LOGI(TAG, "[INIT] Initialized runtime personality table with %d personalities", rc);
    }
    else
    {
        ESP_LOGW(TAG, "[INIT] Error: failed to initialize runtime personality table!");
        // TODO: recovery state
    }

    /* Build physical submodules */
    loadNodeDefaults();

    /* Read the NVS data from flash */
    handleReadCfgNVS();

    /* Append template personalities based on user defined submodules loaded from NVS */
    rc = appendTemplatePersonalities();
    if (rc < 0)
    {
        ESP_LOGW(TAG, "[INIT] Error: appendTemplatePersonalities() failed");
        // TODO: enter recovery state
    }

    /* Initialize the hardware, attach ISR handlers as needed */
    initNodeHardware();

    /* Initialize dynamic internal submodules */
    discoverInternalSubmodules();

    /* Initialize producer library callbacks */
    producerInit(&producerCB);

    /* Initialize router CRC16 callback */
    router_set_crc_callback(crc16_ccitt);

    /* Install ISR service */
    // initGpioIsrService();

    /* Attach GPIO ISR handlers */
    // attachGpioIsrInit();

#ifdef ESP32CYD
    /* Initialize CYD interface */
    initCYD();
    // TODO: move the calls below into the CYD library
    analogSetAttenuation(ADC_11db);
    pinMode(CYD_LDR, INPUT); /* Setup the LDR as an input TODO: this should be a submodule */
#endif

    /* Start FreeRTOS tasks */
    startTaskTWAI();     /* Start the TWAI task */
    startTaskInput();    /* Start the input event task */
    startTaskOutput();   /* Start the output event task */
    startTaskProducer(); /* Start the producer task */
    startTaskConsumer(); /* Start the consumer task */
}

static void pollDirtyFlags(void)
{
    static uint32_t lastCheck = 0; /* retains value, not global */

    const uint32_t now = millis();
    if (now - lastCheck < 2000)
    {
        return;
    }
    lastCheck = now;

    /* Scan for dirty flags */
    for (uint8_t i = 0; i < MAX_SUB_MODULES; i++)
    {
        if (nodeGetSubModule(i)->submod_flags & SUBMOD_FLAG_DIRTY)
        {

            /* Clear all dirty flags before saving */
            for (uint8_t j = 0; j < MAX_SUB_MODULES; j++)
            {
                nodeGetSubModule(j)->submod_flags &= ~SUBMOD_FLAG_DIRTY;
            }

            saveConfigNvs();
            break;
        }
    }
}

/**
 * Reads the physical hardware register for a given GPIO pin to see
 * the exact interrupt type configured in the silicon.
 */
static void print_raw_gpio_register(int pin)
{
    const int target_pin = pin;                             /**< The target GPIO pin number */
    const int raw_int_type = GPIO.pin[target_pin].int_type; /**< Direct read of the interrupt type from the GPIO peripheral struct */

    Serial.print("Raw Register int_type for GPIO ");
    Serial.print(target_pin);
    Serial.print(": ");
    Serial.println(raw_int_type);
}

void loop()
{
    static uint32_t lastCheck;
    static uint32_t last_twai_watchdog_ts;

    uint32_t now = millis();

    /** Check TWAI watchdog */
    if (now - last_twai_watchdog_ts > 1000) 
    {
        last_twai_watchdog_ts = now; /* update last_twai_watchdog_ts */
        node_state_t *ns = nodeGetState();
        uint32_t silence_ms = now - ns->last_rx_ts;

        /* stage 2 */
        if ((ns->bus_watchdog_stage == 1) && 
            (silence_ms > BUS_SILENCE_THRESHOLD_MS)) 
        {
            // Stage 2: second failure → reboot
            ESP_LOGE(TAG, "Bus watchdog: second silence → rebooting");
            esp_restart();
        }

        /* stage 1 */    
        if ((ns->bus_watchdog_stage == 0) && 
            (silence_ms > BUS_SILENCE_THRESHOLD_MS)) 
            {
            // Stage 1: reset TWAI
            twai_request_driver_restart();
            ns->bus_watchdog_stage = 1;
            ns->last_bus_reset_ts = now;
            ESP_LOGW(TAG, "Bus watchdog: TWAI reset due to silence");
        }
    }

    if (now - lastCheck > 500)
    {
        // Serial.printf("ISR Counter %u\n", g_isr_counter);
        gpio_int_type_t intr_type;

        const uint32_t isrCnt = isrGetCounter();

        // int lvl = gpio_get_level(GPIO_NUM_39);
        // // gpio_int_type_t t = gpio_get_intr_type(GPIO_NUM_39);
        // Serial.printf("millis=%u level=%d  ISR=%u \n",
        //               millis(),
        //               gpio_get_level(GPIO_NUM_39),
        //               isrCnt);

        // print_raw_gpio_register(GPIO_NUM_39);
    }

    /** Check for producer save request */
    if (g_routeSaveRequested)
    {
        g_routeSaveRequested = false;
        saveRoutesToNVS();
    }

    /** Check for producer load request */
    if (g_routeLoadRequested)
    {
        g_routeLoadRequested = false;
        loadRoutesFromNVS();
    }

    /** Check for submodule save request, internally rate limited */
    pollDirtyFlags();

    vTaskDelay(pdMS_TO_TICKS(100));
}