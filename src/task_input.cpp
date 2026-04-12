#include "task_input.h"

#include "esp_log.h"
#include "isr_gpio.h"

static const char *TAG = "task_input";

/* ==========================================================================
 *  Private Declarations
 * ==========================================================================*/
static void processGpioEvent(uint8_t subIdx, uint8_t raw);
static void TaskInput(void *pvParameters);

/* Input mode helpers */
static inline void handle_input_momentary(runTime_t *rt, uint8_t value,
                                          uint32_t now, uint8_t subIdx);

static inline void handle_input_toggle(runTime_t *rt, uint8_t value,
                                       uint32_t now, uint8_t subIdx);

static inline void handle_input_latch(runTime_t *rt, uint8_t value,
                                      uint32_t now, uint8_t subIdx);

static inline void handle_input_normal(runTime_t *rt, uint8_t value,
                                       uint32_t now, uint8_t subIdx,
                                       gpio_num_t pin);

/* ==========================================================================
 *  Private functions
 * ==========================================================================*/

/**
 * @brief Process a GPIO event and convert it into value for the producer
 *
 * @param subIdx The submodule index of the GPIO event
 * @param raw The raw value of the GPIO event (0 or 1)
 *
 * This function processes a GPIO event and converts it into a producer
 * subsystem. It first checks if the submodule index is valid and if the pin is
 * enabled. Then it applies the inversion, debouncing and stable state detection
 * according to the mode of the GPIO event. Finally, it updates the runTime
 * struct of the submodule.
 */

static inline void handle_input_momentary(runTime_t *rt, uint8_t value,
                                          uint32_t now, uint8_t subIdx) {

  uint32_t new_value = value ? MOMENTARY_ACTIVE_VALUE : MOMENTARY_RELEASE_VALUE;

  if (rt->valueU32 == new_value) {
    // No logical change → ignore this ISR event
    return;
  }

  uint32_t isr_counter = isrGetCounter();

  rt->valueU32 = new_value;
  rt->last_change_ms = now;

  ESP_LOGD(TAG, "[MOMENTARY] sub=%u isr=%lu valueU32=%lu", subIdx, isr_counter,
           rt->valueU32);
}

static inline void handle_input_toggle(runTime_t *rt, uint8_t value,
                                       uint32_t now, uint8_t subIdx) {
  if (value == GPIO_STATE_HIGH) {
    rt->valueU32 ^= TOGGLE_BIT_MASK; // toggle bit 0
    rt->last_change_ms = now;

    ESP_LOGD(TAG, "[TOGGLE] sub=%u valueU32=%lu", subIdx, rt->valueU32,
             TOGGLE_BIT_MASK);
  }
}

static inline void handle_input_latch(runTime_t *rt, uint8_t value,
                                      uint32_t now, uint8_t subIdx) {
  // Only act on the activation edge (value == HIGH)
  if (value == GPIO_STATE_HIGH) {

    uint32_t new_state;

    // If currently active → switch to release
    if (rt->valueU32 == MOMENTARY_ACTIVE_VALUE) {
      new_state = MOMENTARY_RELEASE_VALUE;
    }
    // If currently released → switch to active
    else {
      new_state = MOMENTARY_ACTIVE_VALUE;
    }

    rt->valueU32 = new_state;
    rt->last_change_ms = now;

    ESP_LOGD(TAG, "[LATCH] sub=%u new_state=%lu", subIdx, new_state);
  }
}

static inline void handle_input_normal(runTime_t *rt, uint8_t value,
                                       uint32_t now, uint8_t subIdx,
                                       gpio_num_t pin) {
  if (value == GPIO_STATE_HIGH) {
    /* Press edge */
    isrGpioUpdatePressStartMs(pin, now);
  } else {
    /* Release edge */
    uint32_t pressDuration = now - isrGpioGetPressStartMs(pin);

    if (pressDuration >= NORMAL_LONG_PRESS_MS) {
      rt->valueU32 = GPIO_LONG_PRESS;
    } else {
      if ((now - isrGpioGetLastClickMs(pin)) < NORMAL_DOUBLE_CLICK_MS) {
        rt->valueU32 = GPIO_DOUBLE_CLICK;
        isrGpioUpdateClickCount(pin, 0);
      } else {
        rt->valueU32 = GPIO_SINGLE_CLICK;
        isrGpioUpdateClickCount(pin, 1);
      }
      isrGpioUpdateLastClickMs(pin, now);
    }

    ESP_LOGD(
        TAG,
        "[NORMAL] sub=%u pressStart=%lu lastClick=%lu duration=%u value=%u",
        subIdx, isrGpioGetPressStartMs(pin), isrGpioGetLastClickMs(pin),
        pressDuration, rt->valueU32);

    rt->last_change_ms = now;
  }

  // ESP_LOGD(TAG, "[NORMAL] sub=%u valueU32=%u", subIdx, rt->valueU32);
}

static void processGpioEvent(uint8_t subIdx, uint8_t raw) {
  // ESP_LOGD(TAG, "subIdx=%u raw=%u isr=%lu", subIdx, raw, isrGetCounter());
  if (SUBMODULE_INDEX_INVALID(subIdx))
    return;

  GET_RUNTIME_OR_RETURN_VOID(subIdx);

  const uint32_t now = millis(); /**< timestamp in ms */
  subModule_t *sub =
      nodeGetActiveSubModule(subIdx); /**< pointer to the submodule */
  const personalityDef_t *p = nodeGetActivePersonality(
      sub->personalityIndex); /**< pointer to the personality definition */
  const uint8_t pinNum = p->gpioPin; /**< GPIO pin number */

  /** Test for valid pin number */
  if (pinNum < GPIO_NUM_MAX) {
    // ESP_LOGD(TAG, "[INPUT] sub=%u pin=%u raw=%u", subIdx, pinNum, raw);

    const gpio_num_t pin = (gpio_num_t)pinNum; /**< GPIO pin */

    const uint8_t debounceMs = sub->config.gpioInput.debounce_ms;
    const uint8_t flags = sub->config.gpioInput.flags;

    const inputModeType_t mode = (inputModeType_t)INPUT_FLAG_GET_MODE(flags);
    const inputInvert_t inv = (inputInvert_t)INPUT_FLAG_GET_INV(flags);

    /* Apply inversion */
    const uint8_t value =
        inv ? !raw
            : raw; /**< Invert raw value if requested, store as output value */

    /* Record raw state */
    isrGpioUpdateRaw(pin, value, now);

    // ESP_LOGV(TAG, "[RAW] pin=%u value=%u lastRaw=%u lastChange=%u",
    //          pinNum,
    //          value,
    //          isrGpioGetRaw((gpio_num_t)pinNum),
    //          isrGpioGetLastChange((gpio_num_t)pinNum));

    // ESP_LOGV(TAG, "[DEBOUNCE] sub=%u mode=%u now=%u lastChange=%u diff=%u
    // debounceMs=%u",
    //          subIdx,
    //          mode,
    //          now,
    //          isrGpioGetLastChange((gpio_num_t)pinNum),
    //          now - isrGpioGetLastChange((gpio_num_t)pinNum),
    //          debounceMs);

    /* Debounce window, skip it if debounce is disabled */
    if (debounceMs != INPUT_DEBOUNCE_DISABLED &&
        ((now - isrGpioGetLastChange(pin)) < debounceMs))
      return;

    /* Stable state detection */
    if (value == isrGpioGetLastStable(pin)) {
      return; // no stable change, nothing to do
    }

    // isrGpio.stableState[pinNum]  = value;
    // isrGpio.lastStableMs[pinNum] = now;

    isrGpioUpdateStable(pin, value, now);
    ESP_LOGD(TAG, "[STABLE] sub=%u value=%u mode=%u now=%u", subIdx, value,
             mode, now);

    /* ============================
     *  MODE DISPATCH
     * ============================ */

    switch (mode) {

      case INPUT_MODE_MOMENTARY:
        handle_input_momentary(rt, value, now, subIdx);
        break;

      case INPUT_MODE_TOGGLE:
        handle_input_toggle(rt, value, now, subIdx);
        break;

      case INPUT_MODE_LATCH: handle_input_latch(rt, value, now, subIdx); break;

      case INPUT_MODE_NORMAL:
        handle_input_normal(rt, value, now, subIdx, pin);
        break;

      default:
        ESP_LOGD(TAG, "[GPIO] sub=%u valueU32=%u", subIdx, rt->valueU32);
        break;
    }

  } else { /* end of pinNum check, non-gpio pin */
    rt->valueU32 = raw;
    rt->last_change_ms = now;
  }
  /* Enqueue input event */
  // ESP_LOGD(TAG, "[INPUT-ENQUEUE] sub=%u valueU32=%u", subIdx, rt->valueU32);
  enqueueInputCmd(INPUT_CMD_GPIO_EVENT, /* this is a GPIO event */
                  subIdx,               /* submodule index */
                  now);

} /* end of processGpioEvent() */

static void TaskInput(void *pvParameters) {
  ESP_LOGI(TAG, "[RTOS] GPIO input task started.");
  gpio_event_t evt;

  for (;;) {
    /* Handle GPIO events */
    if (xQueueReceive(gpioEventQueue, &evt, portMAX_DELAY)) {
      processGpioEvent(evt.subIdx, evt.raw);
    }
    // vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ==========================================================================
 *  Public functions
 * ==========================================================================*/

void startTaskInput() {
  /* Start the input event task*/
  xTaskCreate(TaskInput, "Task Input Events",
              TASK_INPUT_STACK_SIZE,   /* 4096 bytes */
              NULL, tskNormalPriority, /* normal priority */
              &xInputHandle);
}

void enqueueInputCmd(inputCommand_t type, uint8_t index, uint32_t timestamp) {
  inputCommandMsg_t cmd = {
      .type = type, .index = index, .timestamp = timestamp};

  xQueueSend(inputTaskQueue, &cmd, 0);
  ESP_LOGV(TAG, "[INPUT] Input task queue length: %d",
           uxQueueMessagesWaiting(inputTaskQueue));
}
