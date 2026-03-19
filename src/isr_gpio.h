/**
 * @file isr_gpio.h
 * @brief Public API for runtime GPIO ISR management.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "node_state.h"        /**< Provides nodeInfo_t and subModule_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  CONSTANTS
 * ========================================================================== */

/**
 * @brief Maximum GPIO pin index supported by ESP32 family.
 * @note  ESP32 classic supports 0–39.
 */
#define GPIO_PIN_MAX 40

/* ============================================================================
 *  EXTERNAL TABLES
 * ========================================================================== */

extern int8_t gpioToSubIdx[GPIO_PIN_MAX];     /**< Pin → submodule index */
extern bool   gpioIsrEnabled[GPIO_PIN_MAX];   /**< Pin → ISR enabled flag */

/* ============================================================================
 *  PUBLIC API
 * ========================================================================== */

void initGpioIsrService(void);                /**< Install ISR service */
void registerDigitalInputPin(uint8_t pin,
                             uint8_t subIdx); /**< Configure pin + map */
void enableDigitalInputISR(uint8_t pin);      /**< Attach ISR */
void disableDigitalInputISR(uint8_t pin);     /**< Detach ISR */

#ifdef __cplusplus
}
#endif
