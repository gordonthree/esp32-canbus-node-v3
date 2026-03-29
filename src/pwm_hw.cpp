/**
 * @file pwm_hw.cpp
 * @brief LEDC-based PWM hardware control for PWM-capable submodules.
 *
 * @details
 * This module provides minimal hardware-level PWM control using the ESP‑IDF
 * LEDC driver. All PWM uses:
 *   - LEDC low-speed mode
 *   - 13-bit resolution (0–8191 duty)
 *   - One LEDC channel per submodule
 *   - Default frequency of 250 Hz
 *
 * Behavior logic (blink, dimming, patterns) is handled by task_output.cpp.
 */

#include "pwm_hw.h"

#include <Arduino.h>              // optional, safe for now
#include "driver/ledc.h"
#include "node_state.h"           // nodeGetPersonality()
#include "personality_table.h"    // personalityDef_t

/* --------------------------------------------------------------------------
 *  LEDC Resource Tracking Private Variables
 * -------------------------------------------------------------------------- */

static ledc_channel_t channelMap[MAX_SUB_MODULES] = {};
static ledc_timer_t   timerMap[MAX_SUB_MODULES]   = {};

static uint8_t nextChannel = 0;
static uint8_t nextTimer   = 0;

/* LEDC configuration constants */
static constexpr ledc_mode_t      pwmMode       = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t pwmResolution = LEDC_TIMER_13_BIT;
static constexpr uint32_t         pwmDefaultHz  = 250;   // real PWM default

blinkerTracker_t blinkers[LEDC_MAX_TIMERS];
uint8_t pwmPins[LEDC_MAX_TIMERS];       

/* --------------------------------------------------------------------------
 *  Private Helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief Allocate a new LEDC timer.
 */
static ledc_timer_t allocateTimer()
{
    ledc_timer_t timer = static_cast<ledc_timer_t>(nextTimer++);
    return timer;
}

/**
 * @brief Allocate a new LEDC channel.
 */
static ledc_channel_t allocateChannel()
{
    ledc_channel_t ch = static_cast<ledc_channel_t>(nextChannel++);
    return ch;
}


/* --------------------------------------------------------------------------
 *  Public API Implementations
 * -------------------------------------------------------------------------- */

/** @brief Initialize LEDC memory resources */
void pwmHwInit(void)
{
    memset(blinkers, 0, sizeof(blinkers));
    memset(pwmPins, 0, sizeof(pwmPins));
}
void pwmInitChannel(uint8_t index, subModule_t* sub)
{
    const personalityDef_t* p = nodeGetPersonality(sub->personalityIndex);
    if (!p) return;

    const int pin = p->gpioPin;

    /* Allocate LEDC resources */
    ledc_channel_t ch = allocateChannel();
    ledc_timer_t   tm = allocateTimer();

    channelMap[index] = ch;
    timerMap[index]   = tm;

    /* Configure LEDC timer */
    ledc_timer_config_t timerCfg = {};
    timerCfg.speed_mode      = pwmMode;
    timerCfg.timer_num       = tm;
    timerCfg.duty_resolution = pwmResolution;
    timerCfg.freq_hz         = pwmDefaultHz;
    timerCfg.clk_cfg         = LEDC_AUTO_CLK;

    ledc_timer_config(&timerCfg);

    /* Configure LEDC channel */
    ledc_channel_config_t chCfg = {};
    chCfg.speed_mode     = pwmMode;
    chCfg.channel        = ch;
    chCfg.timer_sel      = tm;
    chCfg.intr_type      = LEDC_INTR_DISABLE;
    chCfg.gpio_num       = pin;
    chCfg.duty           = 0;     // task_output will set duty
    chCfg.hpoint         = 0;

    ledc_channel_config(&chCfg);

    Serial.printf("Submod %d: PWM Init (Pin %d, Timer %d, Channel %d)\n",
                  index, pin, tm, ch);
}


void pwmSetDuty(uint8_t index, uint16_t duty)
{
    ledc_channel_t ch = channelMap[index];

    ledc_set_duty(pwmMode, ch, duty);
    ledc_update_duty(pwmMode, ch);
}


void pwmSetFrequency(uint8_t index, uint32_t hz)
{
    ledc_timer_t tm = timerMap[index];

    ledc_set_freq(pwmMode, tm, hz);
}


void pwmStop(uint8_t index)
{
    ledc_channel_t ch = channelMap[index];

    ledc_set_duty(pwmMode, ch, 0);
    ledc_update_duty(pwmMode, ch);
}
