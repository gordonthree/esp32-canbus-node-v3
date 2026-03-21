#pragma once
#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Debounce + timing */
#define INPUT_DEBOUNCE_MIN_MS      (1u)
#define INPUT_DEBOUNCE_MAX_MS      (250u)

/* GPIO output values */
#define GPIO_STATE_LOW             (0u)
#define GPIO_STATE_HIGH            (1u)

/* Momentary mode output values */
#define MOMENTARY_PRESS_VALUE       (10u)
#define MOMENTARY_RELEASE_VALUE     (11u)

/* Normal button timing */
#define NORMAL_LONG_PRESS_MS        (800u)
#define NORMAL_DOUBLE_CLICK_MS      (300u)

/* Toggle and latch mode */
#define TOGGLE_BIT_MASK             (0x01u)
#define GPIO_LATCH_ON               (1U)
#define GPIO_LATCH_OFF              (0U)

/* Normal mode */
#define GPIO_LONG_PRESS             (2U)
#define GPIO_DOUBLE_CLICK           (3U)
#define GPIO_SINGLE_CLICK           (1U)

static const uint32_t LONG_PRESS_MS   = 600;   // hold > 600ms
static const uint32_t DOUBLE_CLICK_MS = 300;   // second click within 300ms

extern bool gpioIsrFired;

extern volatile uint32_t isr_debug_node;
extern volatile uint32_t isr_debug_sub;
extern volatile uint32_t isr_debug_val;
extern volatile uint32_t isr_debug_idx;


/* --------------------------------------------------------------------------
 * Public API for GPIO ISR subsystem
 * -------------------------------------------------------------------------- */


/**
 * @brief Attach an ISR handler to a GPIO pin.
 * @details This function installs the ISR handler for the given GPIO pin.
 * The handler is responsible for calling the sub-module's runtime function
 * when a digital input change is detected.
 * @param pin GPIO pin number
 * @param subIdx Submodule index
 */
void attachDigitalInputISR(uint8_t pin, uint8_t subIdx);

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
