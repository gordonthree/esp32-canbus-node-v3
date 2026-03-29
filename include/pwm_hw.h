/**
 * @file pwm_hw.h
 * @brief Public interface for LEDC-based PWM hardware control.
 *
 * @details
 * This module provides minimal ESP‑IDF LEDC configuration for PWM-capable
 * submodules. All functions here operate strictly at the hardware level:
 *   - Allocate LEDC channels and timers
 *   - Configure 13-bit resolution
 *   - Apply frequency and duty cycle
 *   - Stop PWM output
 *
 * Runtime behavior (blink, strobe, dimming, patterns) is handled entirely
 * by task_output.cpp. This module performs no behavioral logic.
 */

#pragma once
#include <stdint.h>
#include "submodule_types.h"
#include "canbus_project.h"

#ifdef __cplusplus
extern "C" {
#endif

struct blinkerTracker_t {
    uint32_t     freq;              /**< LEDC frequency in Hertz */
    uint8_t      hwPin;             /**< Hardware pin */
    uint8_t      subIdx;            /**< Node submodule index */
    bool         isActive;          /**< Flag to indicate blinker is outputting a signal */
};

/**
 * @brief Initialize a PWM channel for a submodule.
 *
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 *
 * @note
 * This allocates an LEDC channel and timer (if needed), configures 13-bit
 * resolution, and sets the default frequency (250 Hz). Duty is not set here.
 */
void pwmInitChannel(uint8_t index, subModule_t* sub);

/**
 * @brief Set the PWM duty cycle for a submodule.
 *
 * @param index Submodule index.
 * @param duty  Duty cycle in 13-bit range (0–8191).
 */
void pwmSetDuty(uint8_t index, uint16_t duty);

/**
 * @brief Set the PWM frequency for a submodule.
 *
 * @param index Submodule index.
 * @param hz    Frequency in Hz (e.g., 1–10 Hz for blink, 250 Hz for real PWM).
 */
void pwmSetFrequency(uint8_t index, uint32_t hz);

/**
 * @brief Stop PWM output for a submodule.
 *
 * @param index Submodule index.
 */
void pwmStop(uint8_t index);
void pwmHwInit(void);

#ifdef __cplusplus
}
#endif
