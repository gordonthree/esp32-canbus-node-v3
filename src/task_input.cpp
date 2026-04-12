#include "task_input.h"

#include "isr_gpio.h"
#include "esp_log.h"

static const char *TAG = "task_input";

/* ==========================================================================
 *  Private Declarations
 * ==========================================================================*/
static void processGpioEvent(uint8_t subIdx, uint8_t raw);
static void TaskInput(void *pvParameters);

/* ==========================================================================
 *  Private functions
 * ==========================================================================*/

/**
 * @brief Process a GPIO event and convert it into value for the producer
 *
 * @param subIdx The submodule index of the GPIO event
 * @param raw The raw value of the GPIO event (0 or 1)
 *
 * This function processes a GPIO event and converts it into a producer subsystem.
 * It first checks if the submodule index is valid and if the pin is enabled.
 * Then it applies the inversion, debouncing and stable state detection according to the mode of the GPIO event.
 * Finally, it updates the runTime struct of the submodule.
 */

static void processGpioEvent(uint8_t subIdx, uint8_t raw)
{
  // ESP_LOGD(TAG, "subIdx=%u raw=%u isr=%lu", subIdx, raw, isrGetCounter());
  if (SUBMODULE_INDEX_INVALID(subIdx))
    return;

  GET_RUNTIME_OR_RETURN_VOID(subIdx);

  const uint32_t now = millis(); /**< timestamp in ms */
  subModule_t *sub = nodeGetActiveSubModule(subIdx); /**< pointer to the submodule */
  const personalityDef_t *p =
      nodeGetActivePersonality(sub->personalityIndex); /**< pointer to the personality definition */
  const uint8_t pinNum = p->gpioPin;             /**< GPIO pin number */

  /** Test for valid pin number */
  if (pinNum < GPIO_NUM_MAX)
  {
    // ESP_LOGD(TAG, "[INPUT] sub=%u pin=%u raw=%u", subIdx, pinNum, raw);

    const gpio_num_t pin = (gpio_num_t)pinNum; /**< GPIO pin */

    const uint8_t debounceMs = sub->config.gpioInput.debounce_ms;
    const uint8_t flags = sub->config.gpioInput.flags;

    const inputModeType_t mode = (inputModeType_t)INPUT_FLAG_GET_MODE(flags);
    const inputInvert_t inv = (inputInvert_t)INPUT_FLAG_GET_INV(flags);

    /* Apply inversion */
    const uint8_t value = inv ? !raw : raw; /**< Invert raw value if requested, store as output value */

    /* Record raw state */
    isrGpioUpdateRaw(pin, value, now);

    // ESP_LOGV(TAG, "[RAW] pin=%u value=%u lastRaw=%u lastChange=%u",
    //          pinNum,
    //          value,
    //          isrGpioGetRaw((gpio_num_t)pinNum),
    //          isrGpioGetLastChange((gpio_num_t)pinNum));

    // ESP_LOGV(TAG, "[DEBOUNCE] sub=%u mode=%u now=%u lastChange=%u diff=%u debounceMs=%u",
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
    if (value == isrGpioGetLastStable(pin))
    {
      return; // no stable change, nothing to do
    }

    // isrGpio.stableState[pinNum]  = value;
    // isrGpio.lastStableMs[pinNum] = now;

    isrGpioUpdateStable(pin, value, now);
    // ESP_LOGD(TAG, "[STABLE] sub=%u value=%u mode=%u now=%u",
    //          subIdx, value, mode, now);

    /* ============================
     *  MODE: MOMENTARY
     * ============================ */
    if (mode == INPUT_MODE_MOMENTARY)
    {
      uint32_t old_value = rt->valueU32;
      // rt->valueU32 = value ? MOMENTARY_ACTIVE_VALUE
      //                      : MOMENTARY_RELEASE_VALUE;
      // rt->last_change_ms = now;
      uint32_t new_value = value ? MOMENTARY_ACTIVE_VALUE
                           : MOMENTARY_RELEASE_VALUE;

      if (rt->valueU32 == new_value) {
          // No logical change → ignore this ISR event
          return;
      }

      uint32_t isr_counter = isrGetCounter();

      rt->valueU32 = new_value;
      rt->last_change_ms = now;

      ESP_LOGD(TAG, "[MOMENTARY] sub=%u isr=%lu raw=%lu valueU32=%lu",
               subIdx, isr_counter, raw, rt->valueU32);
    }
    /* ============================
     *  MODE: TOGGLE
     * ============================ */
    else if (mode == INPUT_MODE_TOGGLE && value == GPIO_STATE_HIGH)
    {

      rt->valueU32 ^= TOGGLE_BIT_MASK; // toggle bit 0
      rt->last_change_ms = now;
      
      // ESP_LOGD(TAG, "[TOGGLE] sub=%u raw=%u old_value=%lu temp_value=%lu valueU32=%lu macro=0x%08X",
      //          subIdx, raw, old_value, temp_value, rt->valueU32, TOGGLE_BIT_MASK); 
    }
    /* ============================
     *  MODE: LATCH
     * ============================ */
    else if (mode == INPUT_MODE_LATCH)
    {
      rt->valueU32 = value ? GPIO_LATCH_ON : GPIO_LATCH_OFF;
      rt->last_change_ms = now;
      ESP_LOGD(TAG, "[LATCH] sub=%u valueU32=%u",
               subIdx, rt->valueU32);
    }
    /* ============================
     *  MODE: NORMAL BUTTON
     *  (click, double-click, long press)
     * ============================ */
    else if (mode == INPUT_MODE_NORMAL)
    {

      if (value == GPIO_STATE_HIGH)
      {
        isrGpioUpdatePressStartMs(pin, now);
      }
      else
      {
        uint32_t pressDuration = (now - isrGpioGetPressStartMs(pin));

        if (pressDuration >= NORMAL_LONG_PRESS_MS)
        {
          rt->valueU32 = GPIO_LONG_PRESS; // long press
        }
        else
        {
          if ((now - isrGpioGetLastClickMs(pin)) < NORMAL_DOUBLE_CLICK_MS)
          {
            rt->valueU32 = GPIO_DOUBLE_CLICK; // double click
            isrGpioUpdateClickCount(pin, 0);
          }
          else
          {
            rt->valueU32 = GPIO_SINGLE_CLICK; // single click
            isrGpioUpdateClickCount(pin, 1);
          }
          isrGpioUpdateLastClickMs(pin, now);
        }

        rt->last_change_ms = now;
      }
      ESP_LOGI(TAG, "[NORMAL] sub=%u valueU32=%u",
               subIdx, rt->valueU32);
    }
    else
    {
      /* fall-through: if mode doesn't match, we just leave runTime unchanged */
      ESP_LOGD(TAG, "[GPIO] sub=%u valueU32=%u",
               subIdx, rt->valueU32);
    } /* end of mode switch */
  } else {  /* end of pinNum check, non-gpio pin */
    rt->valueU32 = raw;
    rt->last_change_ms = now;
  }
  /* Enqueue input event */
  // ESP_LOGD(TAG, "[INPUT-ENQUEUE] sub=%u valueU32=%u", subIdx, rt->valueU32);
  enqueueInputCmd(
      INPUT_CMD_GPIO_EVENT, /* this is a GPIO event */
      subIdx,               /* submodule index */
      now);

} /* end of processGpioEvent() */

static void TaskInput(void *pvParameters)
{
  ESP_LOGI(TAG, "[RTOS] GPIO input task started.");
  gpio_event_t evt;

  for (;;)
  {
    /* Handle GPIO events */
    if (xQueueReceive(gpioEventQueue, &evt, portMAX_DELAY))
    {
      processGpioEvent(evt.subIdx, evt.raw);
    }
    // vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ==========================================================================
 *  Public functions
 * ==========================================================================*/

void startTaskInput()
{
  /* Start the input event task*/
  xTaskCreate(
      TaskInput,
      "Task Input Events",
      TASK_INPUT_STACK_SIZE, /* 4096 bytes */
      NULL,
      tskNormalPriority, /* normal priority */
      &xInputHandle);
}

void enqueueInputCmd(inputCommand_t type,
                     uint8_t index,
                     uint32_t timestamp)
{
  inputCommandMsg_t cmd = {
      .type = type,
      .index = index,
      .timestamp = timestamp};

  xQueueSend(inputTaskQueue, &cmd, 0);
  ESP_LOGV(TAG, "[INPUT] Input task queue length: %d", uxQueueMessagesWaiting(inputTaskQueue));
}
