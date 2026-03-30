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
  if (subIdx < 0 || subIdx >= MAX_SUB_MODULES)
    return;

  subModule_t *sub = nodeGetSubModule(subIdx);                                 /**< pointer to the submodule */
  const personalityDef_t *p = &runtimePersonalityTable[sub->personalityIndex]; /**< pointer to the personality definition */
  const uint8_t pinNum = p->gpioPin;                                           /**< GPIO pin number */

  /** Test for invalid pin number */
  if (pinNum >= GPIO_NUM_MAX)
    return;

  /** Test if ISR is enabled for this pin */
  if (!isrGpioIsEnabled((gpio_num_t)pinNum))
    return;

  ESP_LOGI(TAG, "sub=%u pin=%u raw=%u", subIdx, pinNum, raw);

  const gpio_num_t pin = (gpio_num_t)pinNum; /**< GPIO pin */
  const uint32_t now = millis();             /**< timestamp in ms */

  const uint8_t debounceMs = sub->config.gpioInput.debounce_ms;
  const uint8_t flags = sub->config.gpioInput.flags;

  const inputModeType_t mode = (inputModeType_t)INPUT_FLAG_GET_MODE(flags);
  const inputInvert_t inv = (inputInvert_t)INPUT_FLAG_GET_INV(flags);

  /* Apply inversion */
  const uint8_t value = inv ? !raw : raw; /**< Invert raw value if requested, store as output value */

  /* Raw change detection */
  isrGpioUpdateRaw(pin, value, now);

  ESP_LOGV(TAG, "[RAW] pin=%u value=%u lastRaw=%u lastChange=%u",
           pinNum,
           value,
           isrGpio.lastRawState[pinNum],
           isrGpio.lastChangeMs[pinNum]);

  ESP_LOGV(TAG, "[DEBOUNCE] now=%u lastChange=%u diff=%u debounceMs=%u",
           now,
           isrGpio.lastChangeMs[pinNum],
           now - isrGpio.lastChangeMs[pinNum],
           debounceMs);

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
  ESP_LOGV(TAG, "[STABLE] sub=%u value=%u mode=%u now=%u",
           subIdx, value, mode, now);

  /* ============================
   *  MODE: MOMENTARY
   * ============================ */
  if (mode == INPUT_MODE_MOMENTARY)
  {
    sub->runTime.valueU32 = value ? MOMENTARY_ACTIVE_VALUE
                                  : MOMENTARY_RELEASE_VALUE;
    sub->runTime.last_change_ms = now;
    ESP_LOGV(TAG, "[MOMENTARY] sub=%u valueU32=%u",
             subIdx, sub->runTime.valueU32);
  }
  /* ============================
   *  MODE: TOGGLE
   * ============================ */
  else if (mode == INPUT_MODE_TOGGLE && value == GPIO_STATE_HIGH)
  {
    sub->runTime.valueU32 ^= TOGGLE_BIT_MASK; // toggle bit 0
    sub->runTime.last_change_ms = now;
    ESP_LOGV(TAG, "[TOGGLE] sub=%u valueU32=%u",
             subIdx, sub->runTime.valueU32);
  }
  /* ============================
   *  MODE: LATCH
   * ============================ */
  else if (mode == INPUT_MODE_LATCH)
  {
    sub->runTime.valueU32 = value ? GPIO_LATCH_ON : GPIO_LATCH_OFF;
    sub->runTime.last_change_ms = now;
    ESP_LOGV(TAG, "[LATCH] sub=%u valueU32=%u",
             subIdx, sub->runTime.valueU32);
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
        sub->runTime.valueU32 = GPIO_LONG_PRESS; // long press
      }
      else
      {
        if ((now - isrGpioGetLastClickMs(pin)) < NORMAL_DOUBLE_CLICK_MS)
        {
          sub->runTime.valueU32 = GPIO_DOUBLE_CLICK; // double click
          isrGpioUpdateClickCount(pin, 0);
        }
        else
        {
          sub->runTime.valueU32 = GPIO_SINGLE_CLICK; // single click
          isrGpioUpdateClickCount(pin, 1);
        }
        isrGpioUpdateLastClickMs(pin, now);
      }

      sub->runTime.last_change_ms = now;
    }
    ESP_LOGI(TAG, "[NORMAL] sub=%u valueU32=%u",
             subIdx, sub->runTime.valueU32);
  }
  else
  {
    /* fall-through: if mode doesn't match, we just leave runTime unchanged */
    ESP_LOGI(TAG, "[GPIO] sub=%u valueU32=%u",
             subIdx, sub->runTime.valueU32);
  } /* end of mode switch */
}

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
    vTaskDelay(pdMS_TO_TICKS(5));
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

void enqueueInputCmd(InputCmdType_t type,
                     uint8_t index,
                     uint8_t edge,
                     uint32_t timestamp)
{
  InputCmd_t cmd = {
      .type = type,
      .index = index,
      .edge = edge,
      .timestamp = timestamp};

  xQueueSend(inputTaskQueue, &cmd, 0);
  ESP_LOGD(TAG, "Input task queue length: %d", uxQueueMessagesWaiting(inputTaskQueue));
}
