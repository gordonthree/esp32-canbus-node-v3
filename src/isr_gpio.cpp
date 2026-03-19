/**
 * @file isr_gpio.cpp
 * @brief Runtime‑registered GPIO interrupt handling for digital inputs.
 */

#include "isr_gpio.h"

/* ============================================================================
 *  LOOKUP TABLES
 * ========================================================================== */

/**
 * @brief Maps GPIO pin → submodule index.
 * @note  Initialized to -1 (unused).
 */
int8_t gpioToSubIdx[GPIO_PIN_MAX] = {
    [0 ... (GPIO_PIN_MAX - 1)] = -1
};

/**
 * @brief Maps GPIO pin → ISR enabled flag.
 * @note  Allows clean runtime enable/disable.
 */
bool gpioIsrEnabled[GPIO_PIN_MAX] = {
    [0 ... (GPIO_PIN_MAX - 1)] = false
};

/* ============================================================================
 *  ISR HANDLER
 * ========================================================================== */

/**
 * @brief Shared ISR handler for all GPIO digital inputs.
 * @param arg  Submodule index (cast from void*).
 */
extern "C" void IRAM_ATTR gpio_isr_handler(void* arg)
{
    const int subIdx = (int)arg;                  /**< Submodule index */
    subModule_t* sub = &node.subModule[subIdx];   /**< Submodule pointer */

    /** Pointer to the personality definition for this sub-module */
    const personalityDef_t* p = getPersonality(sub->personalityId); 
    const uint8_t pin         = p->gpioPin;       /**< Hardware pin */
    const uint32_t now_ms     =
        esp_timer_get_time() / 1000;              /**< Timestamp (ms) */

    const uint8_t state =
        gpio_get_level((gpio_num_t)pin);          /**< Read pin state */

    sub->runTime.valueU32       = state;          /**< Store state */
    sub->runTime.last_change_ms = now_ms;         /**< Store timestamp */
}

/* ============================================================================
 *  PUBLIC API
 * ========================================================================== */

void initGpioIsrService(void)
{
    static bool installed = false;                /**< One‑time guard */
    if (!installed)
    {
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        installed = true;
    }
}

/**
 * @brief Register a digital input pin for ISR handling.
 * @param pin     GPIO pin number.
 * @param subIdx  Submodule index.
 */
void registerDigitalInputPin(uint8_t pin, uint8_t subIdx)
{
    gpioToSubIdx[pin] = subIdx;                   /**< Map pin → submodule */

    gpio_config_t cfg = {};
    cfg.intr_type    = GPIO_INTR_ANYEDGE;         /**< Rising + falling */
    cfg.mode         = GPIO_MODE_INPUT;           /**< Input mode */
    cfg.pin_bit_mask = (1ULL << pin);             /**< Target pin mask */
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;        /**< Default pull‑up */

    gpio_config(&cfg);
}

/**
 * @brief Enable ISR for a digital input pin.
 * @param pin  GPIO pin number.
 */
void enableDigitalInputISR(uint8_t pin)
{
    if (!gpioIsrEnabled[pin] && gpioToSubIdx[pin] >= 0)
    {
        gpio_isr_handler_add(
            (gpio_num_t)pin,
            gpio_isr_handler,
            (void*)gpioToSubIdx[pin]
        );
        gpioIsrEnabled[pin] = true;               /**< Mark enabled */
    }
}

/**
 * @brief Disable ISR for a digital input pin.
 * @param pin  GPIO pin number.
 */
void disableDigitalInputISR(uint8_t pin)
{
    if (gpioIsrEnabled[pin])
    {
        gpio_isr_handler_remove((gpio_num_t)pin);
        gpioIsrEnabled[pin] = false;              /**< Mark disabled */
    }
}

/* EOF */