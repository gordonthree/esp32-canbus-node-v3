#pragma once

/* only load for esp32 hardware */
#ifdef ESP32

/** CYD Cheap Yellow Display pin assignments */
#define CYD_BACKLIGHT_PIN      (21)
#define CYD_LED_RED_PIN        ( 4)
#define CYD_LED_BLUE_PIN       (17)
#define CYD_LED_GREEN_PIN      (16)
#define CYD_LDR_PIN            (34)
#define CYD_SPEAKER_PIN        (26)

/** M5Stamp and M5PICO pin assignments */
#define M5STAMP_BUTTON_COUNT   ( 1)
#define M5STAMP_ARGB_PIN       (27)
#define M5STAMP_ARGB_COUNT     ( 1)
#define M5STAMP_BUTTON_PIN     (39)

#endif /* ESP32 */