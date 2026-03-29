#include "task_output_argb.h"
#include "task_output_gpio.h"
#include "argb_hw.h"

#include "esp_log.h"

/*
 * =========================================================================
 * Constants and macros
 * =========================================================================
 */

static const char *TAG = "task_output_argb";

/*
 * =========================================================================
 * Public functions
 * =========================================================================
 */

void argbRequestStripColor(uint8_t index, uint8_t colorIndex)
{
    argbSetStripColor(index, colorIndex);
}