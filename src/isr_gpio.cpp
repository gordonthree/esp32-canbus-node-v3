#include "isr_gpio.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "submodule_types.h"               // bring in subModule_t
#include "personality_table.h"             // bring in personality table
#include "node_state.h"                    // bring in node object


/* --------------------------------------------------------------------------
 * Internal ISR state (private to this file)
 * -------------------------------------------------------------------------- */

struct IsrGpioState
{
    int8_t  subIdx[GPIO_PIN_MAX];          /**< Pin → submodule index (-1 = unused) */
    bool    enabled[GPIO_PIN_MAX];         /**< Pin → ISR enabled flag */

    uint32_t lastChangeMs[GPIO_PIN_MAX];   /**< Last raw change timestamp */
    uint32_t lastStableMs[GPIO_PIN_MAX];   /**< Last debounced change timestamp */

    uint8_t lastRawState[GPIO_PIN_MAX];    /**< Last raw read */
    uint8_t stableState[GPIO_PIN_MAX];     /**< Last debounced stable state */

    uint8_t edgeMode[GPIO_PIN_MAX];        /**< Edge filter mode */

    IsrGpioState()
    {
        for (int i = 0; i < GPIO_PIN_MAX; ++i) {
            subIdx[i]       = -1;
            enabled[i]      = false;
            lastChangeMs[i] = 0;
            lastStableMs[i] = 0;
            lastRawState[i] = 0;
            stableState[i]  = 0;
            edgeMode[i]     = GPIO_INTR_ANYEDGE;
        }
    }
};

static IsrGpioState isrGpio;    /**< Private ISR state */

/* --------------------------------------------------------------------------
 * ISR service initialization
 * -------------------------------------------------------------------------- */

void initGpioIsrService(void)
{
    static bool installed = false;
    if (!installed) {
        gpio_install_isr_service(0);
        installed = true;
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void registerDigitalInputPin(uint8_t pin, uint8_t subIdx)
{
    if (pin >= GPIO_PIN_MAX) return;
    isrGpio.subIdx[pin] = subIdx;
}

void enableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_PIN_MAX) return;
    isrGpio.enabled[pin] = true;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_ANYEDGE);
}

void disableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_PIN_MAX) return;
    isrGpio.enabled[pin] = false;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
}

void setDigitalInputEdgeMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIO_PIN_MAX) return;
    isrGpio.edgeMode[pin] = mode;
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
}

/* --------------------------------------------------------------------------
 * Shared ISR handler
 * -------------------------------------------------------------------------- */

extern "C" void IRAM_ATTR gpio_isr_handler(void* arg)
{
    const int subIdx = (int)arg;
    if (subIdx < 0 || subIdx >= MAX_SUB_MODULES) return;

    subModule_t* sub = &node.subModule[subIdx];
    const personalityDef_t* p = getPersonality(sub->personalityId);
    const uint8_t pinNum = p->gpioPin;

    if (pinNum >= GPIO_PIN_MAX) return;
    const gpio_num_t pin = (gpio_num_t)pinNum;

    if (!isrGpio.enabled[pinNum])
        return;

    const uint32_t now = esp_timer_get_time() / 1000;
    const uint8_t raw = gpio_get_level(pin);

    const uint8_t debounceMs = sub->config.gpioInput.debounce;  /**< per-pin debounce */

    /* Raw change detection */
    if (raw != isrGpio.lastRawState[pinNum]) {
        isrGpio.lastRawState[pinNum] = raw;
        isrGpio.lastChangeMs[pinNum] = now;
    }

    /* Debounce window */
    if ((now - isrGpio.lastChangeMs[pinNum]) < debounceMs)
        return;

    /* Stable state detection */
    if (raw != isrGpio.stableState[pinNum]) {

        /* Edge filtering */
        const uint8_t mode = isrGpio.edgeMode[pinNum];
        if ((mode == GPIO_INTR_POSEDGE  && raw == 1) ||
            (mode == GPIO_INTR_NEGEDGE  && raw == 0) ||
            (mode == GPIO_INTR_ANYEDGE))
        {
            isrGpio.stableState[pinNum]  = raw;
            isrGpio.lastStableMs[pinNum] = now;

            sub->runTime.valueU32       = raw;
            sub->runTime.last_change_ms = now;
        }
    }
}
