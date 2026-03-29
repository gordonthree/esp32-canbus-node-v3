#include "task_output_argb.h"
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
    ESP_LOGD(TAG, "Set strip %u to palette index %u", index, colorIndex);
    argbSetStripColor(index, colorIndex);
}