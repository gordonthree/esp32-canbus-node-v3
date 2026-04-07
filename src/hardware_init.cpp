/**
 * @file hardware_init.cpp
 * @brief Hardware initialization routines for GPIO and ADC submodules.
 *
 * @details
 * This module provides minimal, ESP‑IDF‑based hardware bring‑up for
 * input and output GPIO pins, as well as analog input pins. All
 * functions except initHardware() are private to this module.
 *
 * Responsibilities:
 *   - Configure digital input pins (pull‑ups, pull‑downs, floating)
 *   - Configure digital output pins (direction only)
 *   - Configure analog input pins (disable digital path)
 *
 * Not handled here:
 *   - PWM hardware (handled in pwm_hw.cpp)
 *   - ARGB hardware (handled in argb_hw.cpp)
 *   - ISR attachment/enabling (handled in isr_gpio.cpp or main.cpp)
 *   - Runtime behavior (task_input.cpp / task_output.cpp)
 */

#include "hardware_init.h"

#include <Arduino.h>           // optional, safe to include for now
#include "driver/gpio.h"       // ESP‑IDF GPIO configuration
#include "node_state.h"        // nodeGetPersonality()
#include "personality_table.h" // personalityDef_t

#include "esp_log.h"

static const char *TAG = "hardware_init";

/* --------------------------------------------------------------------------
 *  Private Function Prototypes
 * -------------------------------------------------------------------------- */

static bool platformCPUTEMP(void); /* Newer ESP32 S3, STM32, maybe others. */
static bool platformESP32(void);   /* All ESP32 variants */
static bool platformAny(void);     /* All platforms */
/**
 * @brief Configure a GPIO pin as a digital input.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initGPIOInput(uint8_t index,
                          const subModule_t *sub,
                          const personalityDef_t *p);

/**
 * @brief Configure a GPIO pin as a digital output.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initGpioOutput(uint8_t index,
                           const subModule_t *sub,
                           const personalityDef_t *p);

/**
 * @brief Configure a GPIO pin for analog input.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initAnalogInput(uint8_t index,
                            const subModule_t *sub,
                            const personalityDef_t *p);

/* Table of internal submodules to be discovered */
static const discoveryEntry_t discoveryTable[] = {
    {INTERNAL_FREE_HEAP, platformESP32}, /* or always true */
    {INTERNAL_WIFI_RSSI, platformESP32},
    {INTERNAL_RTOS_HIGHWATERMARK, platformAny},
    {INTERNAL_INTERNAL_TEMPERATURE, platformCPUTEMP},
    {INTERNAL_RESET_REASON, platformESP32},
    {INTERNAL_BROWNOUT_STATUS, platformESP32},
    {INTERNAL_UPTIME_MS, platformESP32}, /* or always true */
    {INTERNAL_WIFI_CHANNEL, platformESP32},
    {INTERNAL_WIFI_PHY_RATE, platformESP32},
    {INTERNAL_FLASH_SIZE, platformESP32},
    {INTERNAL_CPU_FREQ, platformESP32},
    {INTERNAL_MIN_FREE_HEAP, platformAny},
    {INTERNAL_MAX_FREE_HEAP, platformAny},
    {INTERNAL_CAN_ERROR_COUNTERS, platformAny},
    {INTERNAL_CAN_BUS_STATE, platformAny},
    {INTERNAL_FIRMWARE_VERSION, platformAny},
    {INTERNAL_OTA_PARTITION_INFO, platformESP32},
};

/* Compute the size of the discovery table */
static const size_t discoveryTableSize =
    sizeof(discoveryTable) / sizeof(discoveryEntry_t);

/* --------------------------------------------------------------------------
 *  Private Function Implementations
 * -------------------------------------------------------------------------- */

static bool platformESP32(void)
{
    bool ret = false;
#ifdef ESP32
    ret = true;
#endif
    return ret;
}

static bool platformCPUTEMP(void)
{
    bool ret = false;
#ifdef ESP32S3
    ret = true;
#endif
    return ret;
}

static bool platformAny(void)
{
    return true;
};

static void initGPIOInput(uint8_t index,
                          const subModule_t *sub,
                          const personalityDef_t *p)
{
    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;

    /* Configure pull resistors based on personality flags */
    const uint8_t pull = INPUT_FLAG_GET_PULL(sub->config.gpioInput.flags);
    switch (pull)
    {
    case INPUT_RES_PULLUP:
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        break;

    case INPUT_RES_PULLDOWN:
        cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        break;

    case INPUT_RES_FLOATING:
    default:
        /* both disabled (already set) */
        break;
    }

    gpio_config(&cfg);

    ESP_LOGI(TAG, "[INIT] Submod %d: Digital Input Init (Pin %d)",
             index, p->gpioPin);
}

static void initGpioOutput(uint8_t index,
                           const subModule_t *sub,
                           const personalityDef_t *p)
{

    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&cfg);

    /* Do NOT set the output level here — task_output owns runtime behavior. */

    ESP_LOGI(TAG, "[INIT] Submod %d: Digital Output Init (Pin %d)",
             index, p->gpioPin);
}

static void initAnalogInput(uint8_t index,
                            const subModule_t *sub,
                            const personalityDef_t *p)
{
    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    /* Disable digital path for analog input */
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_DISABLE; // analog mode
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&cfg);

    ESP_LOGI(TAG, "[INIT] Submod %d: Analog Input Init (Pin %d)",
             index, p->gpioPin);
}

/* --------------------------------------------------------------------------
 *  Public Dispatcher
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize hardware for a specific submodule.
 *
 * @details
 * This function dispatches to the appropriate private init routine based
 * on the submodule's introMsgId. Only electrical configuration is performed
 * here; runtime behavior is handled by the input/output tasks.
 *
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
void initHardware(uint8_t index, const subModule_t *sub)
{
    const personalityDef_t *p = nodeGetActivePersonality(sub->personalityIndex);

    /* skip submodules with no personality */
    if (!p)
    {
        ESP_LOGD(TAG, "[INIT] Skipping submodule %d: No Personality", index);
        return;
    }

    switch (sub->introMsgId)
    {

    case INPUT_DIGITAL_GPIO_ID:
        initGPIOInput(index, sub, p);
        break;

    case INPUT_ANALOG_ADC_ID:
        initAnalogInput(index, sub, p);
        break;

    case OUT_GPIO_DIGITAL_ID:
        initGpioOutput(index, sub, p);
        break;

    case OUT_GPIO_DAC_ID:
        // TODO: initAnalogOutput(index, sub);
        break;

        /* PWM handled in pwm_hw.cpp */
        /* ARGB handled in argb_hw.cpp */

    default:
        /* No hardware init required */
        break;
    }
}

void initNodeHardware(void)
{
    /* Loop through the entire submodule table */
    for (uint8_t i = 0; i < MAX_SUB_MODULES; i++)
    {
        const subModule_t *sub = nodeGetActiveSubModule(i);
        if (!sub)
            continue; /* Skip inactive submodule */


        
        if (nodeIsNetworkSubmodule(i) || nodeIsInternalSubmodule(i))
            continue; /* Skip internal and network submodules */

        /* call hardware init function */
        initHardware(i, sub);
    }
}

void discoverInternalSubmodules(void)
{
    /** counter for internal submodules that are auto-created */
    uint8_t count = 0; 

    for (size_t i = 0; i < discoveryTableSize; i++)
    {
        const discoveryEntry_t *d = &discoveryTable[i];

        /* check for and run the probe function, if both return true proceed */
        if (d->probeFn && d->probeFn())
        {
            int idx = addSubmodule(d->personalityId, NULL, 0);

            if (idx < 0)
            {
                ESP_LOGW(TAG,
                         "[INIT] Failed to add internal submodule for personality %u",
                         d->personalityId);
            } else {
                ESP_LOGD(TAG,
                         "[INIT] Added internal submodule for personality %u",
                         d->personalityId);
                count++;
            }
        }
    }

    ESP_LOGI(TAG, "[INIT] Discovered %d internal submodules", count);
}
