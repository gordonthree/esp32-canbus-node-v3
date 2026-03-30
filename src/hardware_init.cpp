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

#include <Arduino.h>              // optional, safe to include for now
#include "driver/gpio.h"          // ESP‑IDF GPIO configuration
#include "node_state.h"           // nodeGetPersonality()
#include "personality_table.h"    // personalityDef_t

#include "esp_log.h"

static const char *TAG = "hardware_init";

/* --------------------------------------------------------------------------
 *  Private Function Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief Configure a GPIO pin as a digital input.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initGPIOInput(uint8_t index, subModule_t* sub);

/**
 * @brief Configure a GPIO pin as a digital output.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initGpioOutput(uint8_t index, subModule_t* sub);

/**
 * @brief Configure a GPIO pin for analog input.
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 */
static void initAnalogInput(uint8_t index, subModule_t* sub);


/* --------------------------------------------------------------------------
 *  Private Function Implementations
 * -------------------------------------------------------------------------- */

static void initGPIOInput(uint8_t index, subModule_t* sub)
{
    const personalityDef_t* p = nodeGetPersonality(sub->personalityIndex);
    if (!p) return;

    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;

    /* Configure pull resistors based on personality flags */
    const uint8_t pull = INPUT_FLAG_GET_PULL(sub->config.gpioInput.flags);
    switch (pull) {
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


static void initGpioOutput(uint8_t index, subModule_t* sub)
{
    const personalityDef_t* p = nodeGetPersonality(sub->personalityIndex);
    if (!p) return;

    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode         = GPIO_MODE_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;

    gpio_config(&cfg);

    /* Do NOT set the output level here — task_output owns runtime behavior. */

    ESP_LOGI(TAG, "[INIT] Submod %d: Digital Output Init (Pin %d)",
                  index, p->gpioPin);
}


static void initAnalogInput(uint8_t index, subModule_t* sub)
{
    const personalityDef_t* p = nodeGetPersonality(sub->personalityIndex);
    if (!p) return;

    const gpio_num_t pin = (gpio_num_t)p->gpioPin;

    /* Disable digital path for analog input */
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode         = GPIO_MODE_DISABLE;   // analog mode
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;

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
void initHardware(uint8_t index, subModule_t* sub)
{
    const personalityDef_t* p = nodeGetPersonality(sub->personalityIndex);
    if (!p) return;

    switch (sub->introMsgId) {

        case INPUT_DIGITAL_GPIO_ID:
            initGPIOInput(index, sub);
            break;

        case INPUT_ANALOG_ADC_ID:
            initAnalogInput(index, sub);
            break;

        case OUT_GPIO_DIGITAL_ID:
            initGpioOutput(index, sub);
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

void initNodeHardware(void) {
    for (uint8_t i = 0; i < nodeGetInfo()->subModCnt; i++) {
        initHardware(i, nodeGetSubModule(i));
    }
}