/** @file personality_table.c */

#include "personality_table.h"

#if defined(M5STAMP) || defined(ESP32PICO) // M5Stamp and ESP32PICO use the same hardware definitions

#include "esp32_defs.h"

static const personalityDef_t personalityTable_0x79C[] = {

    /* ----------------------------------------------------------------------
     * Submodule 0 — ARGB LED Output
     * ---------------------------------------------------------------------- */
    {
        .personalityId = PERS_ARGB_OUTPUT,
        .capabilities  = CAP_RGB,                     /**< ARGB LED strip */

        /* Hardware mapping */
        .gpioPin       = M5STAMP_ARGB_PIN,            /**< Pin 27 */
        .pwmChannel    = 0xFF,                        /**< Not PWM-capable */
        .pwmTimer      = 0xFF,                        /**< Not PWM-capable */
        .isSinkDriver  = false,                       /**< Not relevant */

        /* Data reporting / control */
        .dataMsgId     = 0x210,                       /**< SET_ARGB_STRIP_COLOR */
        .dataMsgDlc    = 6                            /**< nodeID x4 + displayId + colorIndex */
    },

    /* ----------------------------------------------------------------------
     * Submodule 1 — Digital GPIO Input
     * ---------------------------------------------------------------------- */
    {
        .personalityId = PERS_DIGITAL_INPUT,
        .capabilities  = CAP_INPUT,                   /**< Digital input */

        /* Hardware mapping */
        .gpioPin       = M5STAMP_BUTTON_PIN,          /**< Pin 39 */
        .pwmChannel    = 0xFF,                        /**< Not PWM-capable */
        .pwmTimer      = 0xFF,                        /**< Not PWM-capable */
        .isSinkDriver  = false,                       /**< Inputs do not sink/source */

        /* Data reporting */
        .dataMsgId     = 0x711,                       /**< INPUT_DIGITAL_GPIO data */
        .dataMsgDlc    = 8                            /**< 8-byte payload */
    }
};
#endif

