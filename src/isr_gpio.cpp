#include "isr_gpio.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "submodule_types.h"   // bring in subModule_t
#include "personality_table.h" // bring in personality table
#include "node_state.h"        // bring in node object

#include "esp_log.h"

static const char *TAG = "isr_gpio";

/* Get this out of the way early */
extern "C" void IRAM_ATTR gpio_isr_handler(void *arg);


/* --------------------------------------------------------------------------
 * Global state
 * -------------------------------------------------------------------------- */

IsrGpioState isrGpio; /* ISR state */

IsrGpioState::IsrGpioState() /* Constructor */
{
    for (int i = 0; i < GPIO_NUM_MAX; ++i)
    {
        subIdx[i] = -1;
        enabled[i] = false;
        lastChangeMs[i] = 0;
        lastStableMs[i] = 0;
        lastRawState[i] = 0;
        stableState[i] = 0;
        edgeMode[i] = GPIO_INTR_ANYEDGE;

        pressStartMs[i] = 0;
        lastClickMs[i] = 0;
        clickCount[i] = 0;
    }
}

extern "C" volatile uint32_t g_isr_counter = 0;

extern "C" uint32_t isrGetCounter(void)
{
    return g_isr_counter;
}


/* --------------------------------------------------------------------------
 * ISR service initialization
 * -------------------------------------------------------------------------- */

void initGpioIsrService(void)
{
    static bool installed = false; /* Only install the ISR service once */
    if (!installed)
    {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        ESP_LOGI(TAG, "[ISR] ISR service install: %s", esp_err_to_name(err));
        installed = true;
    }

    /* Create GPIO ISR event queue if it doesn't already exist */
    if (gpioEventQueue == NULL)
    {
        gpioEventQueue = xQueueCreate(GPIO_EVENT_QUEUE_LEN, sizeof(gpio_event_t));
    }

    if (gpioEventQueue == NULL)
    {
        ESP_LOGW(TAG, "[ISR] ERROR: Failed to create GPIO event queue!");
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

bool isrGpioIsEnabled(gpio_num_t pin)
{
    return isrGpio.enabled[pin];
}

uint8_t isrGpioGetRaw(gpio_num_t pin)
{
    return isrGpio.lastRawState[pin];
}

uint8_t isrGpioGetStable(gpio_num_t pin)
{
    return isrGpio.stableState[pin];
}

uint32_t isrGpioGetLastChange(gpio_num_t pin)
{
    return isrGpio.lastChangeMs[pin];
}

uint32_t isrGpioGetLastStable(gpio_num_t pin)
{
    return isrGpio.lastStableMs[pin];
}

uint32_t isrGpioGetPressStartMs(gpio_num_t pin)
{
    return isrGpio.pressStartMs[pin];
}

uint32_t isrGpioGetLastClickMs(gpio_num_t pin)
{
    return isrGpio.lastClickMs[pin];
}

uint8_t isrGpioGetClickCount(gpio_num_t pin)
{
    return isrGpio.clickCount[pin];
}

bool isrGpioUpdateRaw(gpio_num_t pin, uint8_t newValue, uint32_t nowMs)
{
    if (isrGpio.lastRawState[pin] != newValue)
    {
        isrGpio.lastRawState[pin] = newValue;
        isrGpio.lastChangeMs[pin] = nowMs;
        return true; // raw state changed
    }
    return false; // no change
}

bool isrGpioUpdateStable(gpio_num_t pin, uint8_t newValue, uint32_t nowMs)
{
    if (isrGpio.stableState[pin] != newValue)
    {
        isrGpio.stableState[pin] = newValue;
        isrGpio.lastStableMs[pin] = nowMs;
        return true; // stable state changed
    }
    return false; // no change
}

bool isrGpioUpdatePressStartMs(gpio_num_t pin, uint32_t nowMs)
{
    isrGpio.pressStartMs[pin] = nowMs;
    return true;
}

bool isrGpioUpdateLastClickMs(gpio_num_t pin, uint32_t nowMs)
{
    isrGpio.lastClickMs[pin] = nowMs;
    return true;
}

bool isrGpioUpdateClickCount(gpio_num_t pin, uint8_t count)
{
    isrGpio.clickCount[pin] = count;
    return true;
}

void attachDigitalInputISR(uint8_t pin, uint8_t subIdx)
{
    if (pin >= GPIO_NUM_MAX)
        return;

    /** * Explicitly configure the GPIO using the ESP-IDF structure.
     * This ensures the pin is set to Input mode and the interrupt type is defined.
     */
    // gpio_config_t io_conf = {};
    // io_conf.intr_type = GPIO_INTR_ANYEDGE; /**< Trigger on both rising and falling */
    // io_conf.pin_bit_mask = (1ULL << pin);
    // io_conf.mode = GPIO_MODE_INPUT;

    const subModule_t *sub = nodeGetActiveSubModule(subIdx);
    if (!sub) {
        ESP_LOGW(TAG, "[ISR] ERROR: Submodule %d not found!", subIdx);
        return;
    }
    const uint8_t inputFlags = sub->config.gpioInput.flags;
    const uint8_t inputResistor = INPUT_FLAG_GET_PULL(inputFlags);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_INPUT;

    if (inputResistor == INPUT_FLAG_PULL_UP) {
        io_conf.pull_up_en =   (gpio_pullup_t)GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE;
    } else if (inputResistor == INPUT_FLAG_PULL_DOWN) {
        io_conf.pull_up_en =   (gpio_pullup_t)GPIO_PULLDOWN_DISABLE;
        io_conf.pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_ENABLE;
    } else if (inputResistor == INPUT_FLAG_PULL_FLOAT) {
        io_conf.pull_up_en =   (gpio_pullup_t)GPIO_PULLDOWN_DISABLE;
        io_conf.pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE;
    }

    // Apply inversion flag to interrupt type
    if (inputFlags & INPUT_FLAG_INVERT) {
        io_conf.intr_type = GPIO_INTR_NEGEDGE;   // active-low button
    } else {
        io_conf.intr_type = GPIO_INTR_POSEDGE;   // active-high button
    }

    Serial.print("DEBUG: io_conf.intr_type = ");
    Serial.println((int)io_conf.intr_type); /**< Cast enum to int to display the raw value */

    gpio_config(&io_conf);

    const char* inverted = 
        inputFlags & INPUT_FLAG_INVERT ? " (inverted)" : " (non-inverted)";

    // Register pin → submodule mapping
    isrGpio.subIdx[pin] = subIdx;

    // Attach the ISR handler
    esp_err_t err = gpio_isr_handler_add((gpio_num_t)pin,
                                         gpio_isr_handler,
                                         (void *)(uintptr_t)subIdx);

    ESP_LOGI(TAG, "[ISR] gpio_isr_handler_add: pin=%u subIdx=%u err=%s %s",
             pin, subIdx, esp_err_to_name(err), inverted);

    esp_err_t err2 = gpio_intr_enable((gpio_num_t)pin);

    ESP_LOGI(TAG, "[ISR] gpio_intr_enable: pin=%u subIdx=%u err=%s",
             pin, subIdx, esp_err_to_name(err2));
}

void enableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_NUM_MAX)
        return;
    isrGpio.enabled[pin] = true;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_ANYEDGE);
}

void disableDigitalInputISR(uint8_t pin)
{
    if (pin >= GPIO_NUM_MAX)
        return;
    isrGpio.enabled[pin] = false;
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
}

void setDigitalInputEdgeMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIO_NUM_MAX)
        return;
    isrGpio.edgeMode[pin] = mode;
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
}

/* --------------------------------------------------------------------------
 * Private static helper functions
 * -------------------------------------------------------------------------- */

static inline IRAM_ATTR uint8_t sample_pin_filtered(gpio_num_t pin)
{
    uint8_t a = gpio_get_level(pin);
    uint8_t b = gpio_get_level(pin);
    uint8_t c = gpio_get_level(pin);

    // Majority vote: 2 out of 3
    return (a & b) | (b & c) | (a & c);
}

/* --------------------------------------------------------------------------
 * Shared ISR handler
 * -------------------------------------------------------------------------- */

extern "C" void IRAM_ATTR gpio_isr_handler(void *arg)
{
    const int subIdx = (int)(uintptr_t)arg;

    if (subIdx < 0 || subIdx >= MAX_SUB_MODULES)
        return;

    g_isr_counter++;

    const subModule_t *sub = nodeGetActiveSubModule(subIdx);
    const personalityDef_t *p = nodeGetActivePersonality(sub->personalityIndex);
    const uint8_t pinNum = p->gpioPin;

    if (pinNum >= GPIO_NUM_MAX)
        return;

    const uint8_t raw = sample_pin_filtered((gpio_num_t)pinNum);
    const uint32_t now = xTaskGetTickCountFromISR();

    gpio_event_t evt = {
        .subIdx = (uint8_t)subIdx,
        .raw = raw};

    BaseType_t hpTaskWoken = pdFALSE;
    xQueueSendFromISR(gpioEventQueue, &evt, &hpTaskWoken); // NEW IDENTIFIER #2
    // Inside your ISR
    GPIO.status_w1tc = (1ULL << pinNum); /**< Write 1 to Clear the interrupt status bit */
    portYIELD_FROM_ISR(hpTaskWoken);
} /* gpio_isr_handler() */
