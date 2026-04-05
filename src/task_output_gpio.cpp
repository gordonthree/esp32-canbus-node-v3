#include "task_output_gpio.h"
#include "task_output_tracker.h"   // for tracker access if needed later
#include "driver/ledc.h"
#include "pwm_hw.h"   // your existing LEDC backend
#include "esp_log.h"

static const char* TAG = "task_output_gpio";

/* --------------------------------------------------------------------------
 * GPIO + PWM backend
 * This file contains ONLY electrical behavior:
 *   - push/pull GPIO
 *   - Hi-Z modes
 *   - open-drain
 *   - inversion
 *   - PWM duty/frequency
 * -------------------------------------------------------------------------- */

void setOutput(subModule_t *sub,
               const personalityDef_t* p,
               bool desiredState)
{
    gpio_num_t pin = (gpio_num_t)p->gpioPin;

    /* Apply logical inversion if required */
    bool electricalState = desiredState;
    if (p->capabilities & CAP_OUTPUT_INVERTED) {
        electricalState = !electricalState;
    }

    /* --------------------------------------------------------------------
     * Runtime reporting:
     * - Simple push‑pull GPIO → report 0/1 only
     * - Complex electrical modes → pack extended runtime bits
     * ------------------------------------------------------------------ */
    if (!(p->capabilities & (CAP_HIZ_OFF | CAP_HIZ_ON | CAP_OUTPUT_INVERTED)))
    {
        /* Simple push‑pull output: ON = 1, OFF = 0 */
        sub->runTime.valueU32 = electricalState ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
    }
    else
    {
        /* Extended runtime encoding for Hi‑Z, open‑drain, inverted logic */
        gpioExtendedRuntime_t rt = { .value = 0 };
        rt.bits.logicalState    = desiredState;
        rt.bits.electricalState = electricalState;
        rt.bits.hizOff          = (p->capabilities & CAP_HIZ_OFF)         ? 1U : 0U;
        rt.bits.hizOn           = (p->capabilities & CAP_HIZ_ON)          ? 1U : 0U;
        rt.bits.inverted        = (p->capabilities & CAP_OUTPUT_INVERTED) ? 1U : 0U;
        rt.bits.openDrain       = (p->capabilities & CAP_OPEN_DRAIN)      ? 1U : 0U;

        sub->runTime.valueU32 = rt.value;
    }

    /* --------------------------------------------------------------------
     * Electrical output behavior:
     * - If electricalState == true → ON behavior
     * - If electricalState == false → OFF behavior
     * - Hi‑Z flags override push‑pull drive
     * ------------------------------------------------------------------ */

    if (electricalState)
    {
        /* ON state: Hi‑Z ON (open‑drain release) */
        if (p->capabilities & CAP_HIZ_ON) {
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_INPUT);   /* float HIGH */
            return;
        }

        /* ON state: push‑pull HIGH */
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, GPIO_LEVEL_HIGH);
        return;
    }

    /* OFF state: Hi‑Z OFF (float LOW) */
    if (p->capabilities & CAP_HIZ_OFF) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);       /* float LOW */
    }
    else {
        /* OFF state: push‑pull LOW */
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, GPIO_LEVEL_LOW);
    }
}


void subOutHelper(const uint8_t index, const bool state)
{
    if (!nodeIsActiveSubmodule(index))
    {
        ESP_LOGW(TAG, "Invalid sub module index %d", index);
        return;
    }
    
    subModule_t *sub = nodeGetSubModule(index);
    const personalityDef_t* p = 
        &runtimePersonalityTable[sub->personalityIndex];

    setOutput(sub, p, state);
}


void gpioApplyState(uint8_t index, uint32_t state)
{
    subModule_t *sub = nodeGetSubModule(index);
    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    setOutput(sub, p, state != 0);
}

void gpioApplyPwmDuty(uint8_t index, uint32_t duty13bit)
{
    /* NEW: thin wrapper around your LEDC backend */
    pwmSetDuty(index, duty13bit);
}

void gpioApplyPwmFreq(uint8_t index, uint32_t freqHz)
{
    /* NEW: thin wrapper around your LEDC backend */
    pwmSetFrequency(index, freqHz);
}

/* Existing setOutput() and subOutHelper() move here unchanged */
