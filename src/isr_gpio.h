#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Debounce + timing */
#define INPUT_DEBOUNCE_MIN_MS      (1u)
#define INPUT_DEBOUNCE_MAX_MS      (250u)

/* GPIO output values */
#define GPIO_STATE_LOW             (0u)
#define GPIO_STATE_HIGH            (1u)


/* Normal button timing */
#define NORMAL_LONG_PRESS_MS        (800u)
#define NORMAL_DOUBLE_CLICK_MS      (300u)

/* Toggle and latch mode */
#define TOGGLE_BIT_MASK             (0x01u)
#define GPIO_LATCH_ON               (1U)
#define GPIO_LATCH_OFF              (0U)



/* Event Queue constants */
#define GPIO_EVENT_QUEUE_LEN        (32U)
typedef struct {
    uint8_t  subIdx;
    uint8_t  raw;
} gpio_event_t;

/* --------------------------------------------------------------------------
 * Private API for GPIO ISR subsystem
 * -------------------------------------------------------------------------- */
 static inline IRAM_ATTR uint8_t sample_pin_filtered(gpio_num_t pin);


/* --------------------------------------------------------------------------
 * Public ISR state structure (previously private)
 * -------------------------------------------------------------------------- */
struct IsrGpioState
{
    int8_t  subIdx[GPIO_NUM_MAX];
    bool    enabled[GPIO_NUM_MAX];

    uint32_t lastChangeMs[GPIO_NUM_MAX];
    uint32_t lastStableMs[GPIO_NUM_MAX];

    uint8_t lastRawState[GPIO_NUM_MAX];
    uint8_t stableState[GPIO_NUM_MAX];

    uint8_t edgeMode[GPIO_NUM_MAX];

    uint32_t pressStartMs[GPIO_NUM_MAX];
    uint32_t lastClickMs[GPIO_NUM_MAX];
    uint8_t  clickCount[GPIO_NUM_MAX];

    IsrGpioState();
};

/* --------------------------------------------------------------------------
 * Expose the globals
 * -------------------------------------------------------------------------- */

extern QueueHandle_t gpioEventQueue;
extern IsrGpioState isrGpio;


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
