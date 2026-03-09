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

#define ARGB_511C_PIN          (19)
#define ARGB_511C_CNT          (12)

typedef enum {
    ESP32_PWM_CHANNEL_0 = 0,
    ESP32_PWM_CHANNEL_1,
    ESP32_PWM_CHANNEL_2,
    ESP32_PWM_CHANNEL_3,
    ESP32_PWM_CHANNEL_4,
    ESP32_PWM_CHANNEL_5,
    ESP32_PWM_CHANNEL_6,
    ESP32_PWM_CHANNEL_7
} esp32_pwm_channel_t;

typedef enum {
    ESP32_PWM_TIMER_0 = 0,
    ESP32_PWM_TIMER_1,
    ESP32_PWM_TIMER_2,
    ESP32_PWM_TIMER_3,
} esp32_pwm_timer_t;

#endif /* ESP32 */