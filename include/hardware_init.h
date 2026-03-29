/**
 * @file hardware_init.h
 * @brief Public interface for hardware initialization dispatcher.
 *
 * @details
 * This header exposes a single function, initHardware(), which performs
 * minimal ESP‑IDF‑based electrical configuration for supported submodule
 * types. All specific initialization routines (GPIO input, GPIO output,
 * analog input) are private to hardware_init.cpp.
 *
 * Responsibilities of initHardware():
 *   - Look up the submodule's personality
 *   - Dispatch to the appropriate private init function
 *   - Perform only electrical configuration (no runtime behavior)
 *
 * Not handled here:
 *   - PWM hardware (pwm_hw.cpp)
 *   - ARGB hardware (argb_hw.cpp)
 *   - ISR attachment/enabling (isr_gpio.cpp or main.cpp)
 *   - Runtime behavior (task_input.cpp / task_output.cpp)
 */

#pragma once
#include <stdint.h>
#include "submodule_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *  Public Dispatcher
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize hardware for a specific submodule.
 *
 * @param index Submodule index.
 * @param sub   Pointer to submodule definition.
 *
 * @note
 * This function performs only electrical configuration. Runtime behavior
 * (debouncing, ISR handling, output control, PWM, ARGB, etc.) is handled
 * by other modules.
 */
void initHardware(uint8_t index, subModule_t* sub);

/** @brief Initialize all submodule hardware */
void initNodeHardware(void);

#ifdef __cplusplus
}
#endif
