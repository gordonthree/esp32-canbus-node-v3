#include "consumer_handler.h"
#include "node_state.h"         /* nodeIsValidSubmodule */
#include "colorpalette.h"       /* SystemPalette */
#include "esp_log.h"

static const char *TAG = "consumer_display";


static void handleColorCommand(const can_msg_t *msg);
static void setDisplayMode(can_msg_t *msg, uint8_t displayMode);

/**
 * @brief Handles color commands from the gateway node.
 *
 * @param msg The TWAI message containing the color command.
 *
 */
static void handleColorCommand(const can_msg_t *msg)
{
    const uint8_t subIdx     = msg->data[4];
    const uint8_t colorIndex = msg->data[5];

    if (!nodeIsValidSubmodule(subIdx)) return;

    if (colorIndex >= COLOR_PALETTE_SIZE) return;

    subModule_t *sub = nodeGetSubModule(subIdx);
    const PaletteColor* c = &SystemPalette[colorIndex];

    if (!c) return;  /* exit if invalid color index */

    /* Producer: Update the color value in runTime */
    sub->runTime.valueU32       = packRgb(c->R, c->G, c->B);
    sub->runTime.last_change_ms = millis();

  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_COLOR_INDEX,
      subIdx,
      (uint32_t)colorIndex
  );  

  ESP_LOGI(TAG, "Selecting color index %d for display %d", colorIndex, subIdx);
}


static void setDisplayMode(can_msg_t *msg, uint8_t displayMode) {
  const uint8_t subIdx = msg->data[4]; /* display ID */

  if (!nodeIsValidSubmodule(subIdx)) return;
  
  subModule_t *sub = nodeGetSubModule(subIdx); /* get submodule pointer */

  /* Producer: Update the display mode in runTime */
  sub->runTime.valueU32       = displayMode;
  sub->runTime.last_change_ms = millis();

  enqueueOutputCmd(
    OUTPUT_CMD_SET_DISPLAY_MODE,
    subIdx,
    (uint32_t)displayMode
  );  

  ESP_LOGI(TAG, "Display %d Mode: %d\n", subIdx, displayMode);
}


void handleDisplayCommands(can_msg_t *msg)
{
    switch (msg->identifier)
    {
        case SET_DISPLAY_OFF_ID:          // set display off
            setDisplayMode(msg, DISPLAY_MODE_OFF);
            break;

        case SET_DISPLAY_ON_ID:           // set display on
            setDisplayMode(msg, DISPLAY_MODE_ON);
            break;

        case SET_DISPLAY_FLASH_ID:        // flash display backlight
            setDisplayMode(msg, DISPLAY_MODE_FLASH);
            break;

        case SET_ARGB_STRIP_COLOR_ID:     /* set ARGB color */
            handleColorCommand(msg);      /* byte 4 is the sub module index, byte 5 is the color index */
            break;

        default:
            /* ID is in 0x200–0x2FF but not currently handled */
            break;
    }
}
