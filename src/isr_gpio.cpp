#include "isr_gpio.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "submodule_types.h"               // bring in subModule_t
#include "personality_table.h"             // bring in personality table
#include "node_state.h"                    // bring in node object


/* Get this out of the way early */
extern "C" void IRAM_ATTR gpio_isr_handler(void* arg);
bool gpioIsrFired  = false;

volatile uint32_t isr_debug_node = 0;
volatile uint32_t isr_debug_sub  = 0;
volatile uint32_t isr_debug_val  = 0;
volatile uint32_t isr_debug_idx  = 0;

/* --------------------------------------------------------------------------
 * Internal ISR state (private to this file)
 * -------------------------------------------------------------------------- */

struct IsrGpioState
{
    int8_t  subIdx[GPIO_NUM_MAX];          /**< Pin → submodule index (-1 = unused) */
    bool    enabled[GPIO_NUM_MAX];         /**< Pin → ISR enabled flag */

    uint32_t lastChangeMs[GPIO_NUM_MAX];   /**< Last raw change timestamp */
    uint32_t lastStableMs[GPIO_NUM_MAX];   /**< Last debounced change timestamp */

    uint8_t lastRawState[GPIO_NUM_MAX];    /**< Last raw read */
    uint8_t stableState[GPIO_NUM_MAX];     /**< Last debounced stable state */

    uint8_t edgeMode[GPIO_NUM_MAX];        /**< Edge filter mode */

    uint32_t pressStartMs[GPIO_NUM_MAX];   /**< When the button was pressed */
    uint32_t lastClickMs[GPIO_NUM_MAX];    /**< Last completed click */
    uint8_t  clickCount[GPIO_NUM_MAX];     /**< Click counter for multi-click */

    IsrGpioState()
    {
        for (int i = 0; i < GPIO_NUM_MAX; ++i) {
            subIdx[i]       = -1;
            enabled[i]      = false;
            lastChangeMs[i] = 0;
            lastStableMs[i] = 0;
            lastRawState[i] = 0;
            stableState[i]  = 0;
            edgeMode[i]     = GPIO_INTR_ANYEDGE;

            pressStartMs[i] = 0;
            lastClickMs[i]  = 0;
            clickCount[i]   = 0;
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
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        Serial.printf("ISR service install: %s\n", esp_err_to_name(err));
        installed = true;
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void attachDigitalInputISR(uint8_t pin, uint8_t subIdx)
{
    if (pin >= GPIO_NUM_MAX) return;

    /** * Explicitly configure the GPIO using the ESP-IDF structure.
     * This ensures the pin is set to Input mode and the interrupt type is defined.
     */
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_ANYEDGE; /**< Trigger on both rising and falling */
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode         = GPIO_MODE_INPUT;

    gpio_config(&io_conf);

    // Register pin → submodule mapping
    isrGpio.subIdx[pin] = subIdx;

    // Attach the ISR handler
    esp_err_t err = gpio_isr_handler_add((gpio_num_t)pin,
                                         gpio_isr_handler,
                                         (void*)(uintptr_t)subIdx);

    Serial.printf("[ISR] attachDigitalInputISR: pin=%u subIdx=%u err=%s\n",
                  pin, subIdx, esp_err_to_name(err));

    esp_err_t err2 = gpio_intr_enable((gpio_num_t)pin);

    Serial.printf("[ISR] gpio_intr_enable: pin=%u subIdx=%u err=%s\n",
                  pin, subIdx, esp_err_to_name(err2));

}

void enableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_NUM_MAX) return;
    isrGpio.enabled[pin] = true;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_ANYEDGE);
}

void disableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_NUM_MAX) return;
    isrGpio.enabled[pin] = false;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
}

void setDigitalInputEdgeMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIO_NUM_MAX) return;
    isrGpio.edgeMode[pin] = mode;
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
}

/* --------------------------------------------------------------------------
 * Shared ISR handler
 * -------------------------------------------------------------------------- */

extern "C" void IRAM_ATTR gpio_isr_handler(void* arg)
{
    const int subIdx = (int)(uintptr_t)arg;

    if (subIdx < 0 || subIdx >= MAX_SUB_MODULES)
        return;

    subModule_t* sub          = &node.subModule[subIdx];
    const personalityDef_t* p = &g_personalityTable[sub->personalityIndex];
    const uint8_t pinNum      = p->gpioPin;

    // sub->runTime.valueU32 = 99;   // impossible value
    // sub->runTime.last_published_value = 123456789;  // impossible value
    // sub->runTime.last_change_ms = xTaskGetTickCountFromISR();

    // Export debug info safely
    // isr_debug_idx  = subIdx;
    // isr_debug_node = sub->runTime.last_published_value;
    // isr_debug_sub  = (uint32_t)sub;
    // isr_debug_val  = sub->runTime.valueU32;
    
    

    // return; // early return

    if (pinNum >= GPIO_NUM_MAX)
        return;

    if (!isrGpio.enabled[pinNum])
        return;

    const gpio_num_t pin         = (gpio_num_t)pinNum;
    // const uint32_t now           = esp_timer_get_time() / 1000;
    const uint32_t now           = xTaskGetTickCountFromISR();
    const uint8_t raw            = gpio_get_level(pin);

    const uint8_t debounceMs     = sub->config.gpioInput.debounce_ms;
    const uint8_t flags          = sub->config.gpioInput.flags;

    const inputModeType_t mode   = (inputModeType_t)INPUT_FLAG_GET_MODE(flags);
    const inputInvert_t   inv    = (inputInvert_t)INPUT_FLAG_GET_INV(flags);


    /* Apply inversion */
    const uint8_t value = inv ? !raw : raw;

    /* Raw change detection */
    if (value != isrGpio.lastRawState[pinNum]) {
        isrGpio.lastRawState[pinNum] = value;
        isrGpio.lastChangeMs[pinNum] = now;
    }

    /* Debounce window */
    if ((now - isrGpio.lastChangeMs[pinNum]) < debounceMs)
        return;

    /* Stable state detection */
    if (value != isrGpio.stableState[pinNum]) {

        isrGpio.stableState[pinNum]  = value;
        isrGpio.lastStableMs[pinNum] = now;

        /* ============================
         *  MODE: MOMENTARY
         * ============================ */
        if (mode == INPUT_MODE_MOMENTARY) {
            sub->runTime.valueU32       = value ? MOMENTARY_PRESS_VALUE 
                                                : MOMENTARY_RELEASE_VALUE;
            sub->runTime.last_change_ms = now;
            return;
        }

        /* ============================
         *  MODE: TOGGLE
         * ============================ */
        if (mode == INPUT_MODE_TOGGLE && value == GPIO_STATE_HIGH) {
            sub->runTime.valueU32 ^= TOGGLE_BIT_MASK;   // toggle bit 0
            sub->runTime.last_change_ms = now;
            return;
        }

        /* ============================
         *  MODE: LATCH
         * ============================ */
        if (mode == INPUT_MODE_LATCH) {
            sub->runTime.valueU32       = value ? GPIO_LATCH_ON : GPIO_LATCH_OFF;
            sub->runTime.last_change_ms = now;
            return;
        }

        /* ============================
         *  MODE: NORMAL BUTTON
         *  (click, double-click, long press)
         * ============================ */
        if (mode == INPUT_MODE_NORMAL) {

            if (value == GPIO_STATE_HIGH) {
                isrGpio.pressStartMs[pinNum] = now;
            } else {
                uint32_t pressDuration = now - isrGpio.pressStartMs[pinNum];

                if (pressDuration >= LONG_PRESS_MS) {
                    sub->runTime.valueU32 = GPIO_LONG_PRESS;   // long press
                } else {
                    if ((now - isrGpio.lastClickMs[pinNum]) < DOUBLE_CLICK_MS) {
                        sub->runTime.valueU32 = GPIO_DOUBLE_CLICK;   // double click
                        isrGpio.clickCount[pinNum] = 0;
                    } else {
                        sub->runTime.valueU32 = GPIO_SINGLE_CLICK;   // single click
                        isrGpio.clickCount[pinNum] = 1;
                    }
                    isrGpio.lastClickMs[pinNum] = now;
                }

                sub->runTime.last_change_ms = now;
            }
        }
    }
} /* gpio_isr_handler() */

