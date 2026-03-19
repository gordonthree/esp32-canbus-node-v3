#pragma once
#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_MAX 39

/* --------------------------------------------------------------------------
 * Public API for GPIO ISR subsystem
 * -------------------------------------------------------------------------- */

/**
 * @brief Register a GPIO pin with the ISR system.
 * @param pin     GPIO pin number
 * @param subIdx  Submodule index
 */
void registerDigitalInputPin(uint8_t pin, uint8_t subIdx);

/**
 * @brief Enable ISR for a given GPIO pin.
 * @param pin GPIO pin number
 */
void enableDigitalInputISR(uint8_t pin);

/**
 * @brief Disable ISR for a given GPIO pin.
 * @param pin GPIO pin number
 */
void disableDigitalInputISR(uint8_t pin);

/**
 * @brief Set edge filtering mode for a pin.
 * @param pin  GPIO pin number
 * @param mode One of: GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE, GPIO_INTR_ANYEDGE
 */
void setDigitalInputEdgeMode(uint8_t pin, uint8_t mode);

/**
 * @brief Install the ISR service (must be called once at startup).
 */
void initGpioIsrService(void);

#ifdef __cplusplus
}
#endif
