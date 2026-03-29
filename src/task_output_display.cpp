#include "task_output_display.h"
#include "task_output_gpio.h"      // calls into electrical backend
#include "task_output_tracker.h"   // future: flash/strobe integration

/* --------------------------------------------------------------------------
 * Display semantic layer
 * This file interprets display modes and maps them to electrical actions.
 * It does NOT contain GPIO/PWM code directly.
 * -------------------------------------------------------------------------- */

void displayApplyMode(uint8_t index, uint32_t mode)
{
    subModule_t *sub = nodeGetSubModule(index);
    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    switch (mode) {

        case DISPLAY_MODE_OFF:
            gpioApplyState(index, 0);
            break;

        case DISPLAY_MODE_ON:
            gpioApplyState(index, 1);
            break;

        case DISPLAY_MODE_FLASH:
            /* TODO: integrate with tracker or PWM backend after refactor */
            break;

        case DISPLAY_MODE_CLEAR:
            /* TODO: define semantic meaning (backlight? panel?) */
            break;

        default:
            break;
    }
}
