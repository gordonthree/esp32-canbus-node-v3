#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "task_twai.h"
#include "task_producer.h"
#include "task_input.h"
#include "freertos.h"
#include "submodule_types.h"
#include "node_state.h"
#include "personality_table.h"
#include "can_producer.h"
#include "can_dispatch.h"
#include "wifi_hw.h"
#include "consumer_handler.h"
#include "isr_gpio.h" /* for attachDigitalInputISR() */
#include "esp_log.h"

static const char *TAG = "task_producer";

/* =========================================================================
 *  Private declarations
 * ========================================================================= */

static void updateSubModules();
// static void handleProducerTick(uint32_t now);
static void readCydLdr();
static void managePeriodicMessages();
static void TaskProducer(void *pvParameters);
static void handleInputCommand(inputCommandMsg_t *cmd);
static void internalSubmoduleTick(uint32_t now);
static void canHealthLog(const canHealth_t *h);
static void producerPublishEvent(producer_event_t *evt);
static void producerPublish(const uint16_t msgId, const uint8_t *payload, uint8_t dlc);

/* =========================================================================
 *  Private functions
 * ========================================================================= */

static void canHealthLog(const canHealth_t *h)
{
  ESP_LOGI("CANHEALTH",
           "TEC:%d REC:%d BE:%d AL:%d RXmiss:%d RXovr:%d TXfail:%d "
           "state:%d BOFF:%u EPASS:%u RST:%u RXfull:%u TXdrop:%u",
           h->tx_error_count,
           h->rx_error_count,
           h->bus_error_count,
           h->arb_lost_count,
           h->rx_missed_count,
           h->rx_overrun_count,
           h->tx_failed_count,
           h->controller_state,
           h->bus_off_events,
           h->error_passive_events,
           h->recovery_resets,
           h->rx_queue_full_events,
           h->tx_queue_dropped_events);
}

static void updateSubModules()
{
  /* reconcile submodule count */
  nodeGetActiveSubModuleCount();

  for (int i = 0; i < MAX_SUB_MODULES; i++)
  {
    subModule_t *sub = nodeGetActiveSubModule(i);

    if (!sub)
      continue;

    /* Lookup the personality */
    const personalityDef_t *p = nodeGetActivePersonality(sub->personalityIndex);

    if (!p)
    {
      ESP_LOGE(TAG, "[PRODUCER] Personality lookup failed for index %u", i);
      continue;
    }

    /* Lookup the pin */
    uint8_t myPin = p->gpioPin;

    /* generic storage for value */
    uint32_t value = 0;

    /* skip disabled submodules */
    if (sub->submod_flags & SUBMOD_FLAG_DISABLED)
      continue;

    switch (sub->personalityId)
    {
    /* Addressable RGB output*/
    case PERS_ARGB_OUTPUT:
    { // TODO: Figure out how to read NeoPixelBus status?
      break;
    }
    /* Analog RGBW output */
    case PERS_RGBW_OUTPUT:
    { // TODO
      break;
    }
    /* Analog DAC output */
    case PERS_ANA_OUTPUT:
    { // TODO: Read DAC output value?
      break;
    }
    /* Digital GPIO personalities */
    case PERS_GPIO_OUTPUT:
    case PERS_GPIO_INPUT:
    { // sub->runTime.valueU32 = digitalRead(myPin); /**< Capture digital value */
      break;
    }
      /* Analog ADC input */
    case PERS_ANALOG_INPUT:
    {
      value = (uint32_t)analogRead(myPin); /**< Capture analog value */
      break;
    }

    case INTERNAL_FREE_HEAP:
    {
      value = (uint32_t)xPortGetFreeHeapSize();
      break;
    }

    case INTERNAL_WIFI_RSSI:
    {
      int rssi = wifiGetRssi(); /* get RSSI */
      if (rssi < 0)
        rssi = -rssi; /* absolute value */
      value = (uint32_t)rssi;
      break;
    }

    case INTERNAL_RTOS_HIGHWATERMARK:
    {
      value = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
      break;
    }

    case INTERNAL_INTERNAL_TEMPERATURE:
    {
      value = (uint32_t)temperatureRead();
      break;
    }

    case INTERNAL_RESET_REASON:
    {
      value = (uint32_t)esp_reset_reason();
      break;
    }

    case INTERNAL_BROWNOUT_STATUS:
    { // value = (uint32_t)esp_brownout_get_status();
      break;
    }
    case INTERNAL_FLASH_SIZE:
    {
      // value = (uint32_t)esp_flash_get_size();
      value = (16 * 1024 * 1024);
      break;
    }

    case INTERNAL_UPTIME_MS:
    {
      uint64_t uptime_us = esp_timer_get_time();
      uint32_t uptime_ms = uptime_us / 1000;
      value = uptime_ms;
      break;
    }

    case INTERNAL_WIFI_CHANNEL:
    {
      // wifi_ap_record_t ap;
      // esp_wifi_sta_get_ap_info(&ap);
      // uint8_t channel = ap.primary;
      // value = (uint32_t)channel;
      // TODO: implement actual function
      value = 1;
      break;
    }

    case INTERNAL_WIFI_PHY_RATE:
    {
      // wifi_ap_record_t ap;
      // esp_wifi_sta_get_ap_info(&ap);
      // uint32_t phy_rate = ap.phy_11b; // or 11g/11n flags
      // value = phy_rate;
      // TODO: implement actual function
      value = 11000000;
      break;
    }

    case INTERNAL_CPU_FREQ:
    {
      // value = (uint32_t)esp_clk_cpu_freq();
      value = 240000000; // TODO: implement actual function
      break;
    }

    case INTERNAL_VREF_VOLTAGE:
    {
      // value = (uint32_t)readInternalVref();
      value = 2048; // TODO: implement actual function
      break;
    }

    case INTERNAL_CAN_ERROR_COUNTERS:
    {
      // TODO: implement actual function
      value = 0x14;
      break;
    }

    case INTERNAL_CAN_BUS_STATE:
    {
      // TODO: implement actual function
      value = 0xF00F;
      break;
    }

    case INTERNAL_FIRMWARE_VERSION:
    {
      // TODO: implement actual function
      value = 0x12345678;
      break;
    }

    case INTERNAL_OTA_PARTITION_INFO:
    {
      // TODO: implement actual function
      value = 0xDEADBEEF;
      break;
    }

    default:
    {
      ESP_LOGW(TAG, "[PRODUCER] Ignoring unknown personality %u", sub->personalityId);
      // Unknown virtual personality — ignore safely
      continue;
    }
    } /* end of switch-case */

    sub->runTime.valueU32 = value;          /* update value */
    sub->runTime.last_change_ms = millis(); /* update timestamp */
  } /* end of for loop */
}

/**
 * @brief Publish a message to the CAN bus and perform local loopback
 *
 * @param[in] msgId   CAN message ID
 * @param[in] payload Pointer to message payload data
 * @param[in] dlc     Data length code
 */
static void producerPublish(const uint16_t msgId, const uint8_t *payload, uint8_t dlc)
{
    // 1. Send to CAN hardware
    canEnqueueMessage(msgId, payload, dlc);

    // 2. Local loopback
    enqueueLoopback(msgId, payload, dlc);
}

static void producerPublishEvent(producer_event_t *evt)
{
  const uint32_t now = millis();
  const uint8_t subIdx = evt->sub_idx;

  /* Validate submodule index */
  subModule_t *sub = nodeGetActiveSubModule(subIdx);
  if (!sub)
  {
    ESP_LOGE(TAG, "[PRODUCER] Invalid or inactive submodule at index %u", subIdx);
    return;
  }

  /* Safe personality lookup */
  const personalityDef_t *p = nodeGetActivePersonality(sub->personalityIndex);
  if (!p)
  {
    ESP_LOGE(TAG, "[PRODUCER] Personality lookup failed for sub %u", subIdx);
    return;
  }

  /* Get our nodeID */
  const uint32_t nodeID = nodeGetNodeID();

  uint8_t dlc = p->dataMsgDlc;
  if (dlc < CAN_DATAMSG_MIN_DLC)
    dlc = CAN_DATAMSG_MIN_DLC; /**< enforce minimum DLC for producer messages 6-bytes */
  if (dlc > CAN_MAX_DLC)
    dlc = CAN_MAX_DLC; /**< enforce maximum DLC */

  uint8_t payload[CAN_MAX_DLC] = {0}; /**< zero-initialize */

  /* Node ID (big-endian) */
  payload[0] = (nodeID >> BYTE_SHIFT3) & BYTE_MASK;
  payload[1] = (nodeID >> BYTE_SHIFT2) & BYTE_MASK;
  payload[2] = (nodeID >> BYTE_SHIFT) & BYTE_MASK;
  payload[3] = nodeID & BYTE_MASK;

  /* Submodule index */
  payload[4] = evt->sub_idx;

  const uint32_t valueU32 = evt->value;

  if (valueU32 <= 0xFF)
  { /* Single byte value, send it in position 5 */
    payload[5] = valueU32 & BYTE_MASK;
  }
  else if (valueU32 <= 0xFFFF)
  { /* Two-byte value */
    payload[5] = (valueU32 >> BYTE_SHIFT) & BYTE_MASK;
    payload[6] = valueU32 & BYTE_MASK;
  }
  else
  {                                                     /* Three-byte value */
    payload[5] = (valueU32 >> BYTE_SHIFT2) & BYTE_MASK; /* shift 16 bits */
    payload[6] = (valueU32 >> BYTE_SHIFT) & BYTE_MASK;  /* shift 8 bits */
    payload[7] = valueU32 & BYTE_MASK;
  }

  /* Send message */
  producerPublish(p->dataMsgId, payload, dlc);

  ESP_LOGV(TAG, "[PRODUCER] Tx: sub %u msg 0x%03X dlc %u val %u",
           subIdx, p->dataMsgId, dlc, valueU32);
}

static void readCydLdr()
{
#ifndef ESP32CYD
  return;
#else
  const uint8_t oversample_count = 16; /* Samples for precision */
  const uint8_t ldr_submod_idx = 5;    /* TODO: Make this configurable */

  uint32_t raw_accumulator = 0;

  /* Perform over-sampling on pin 34 */
  ESP_LOGD(TAG, "CYD LDR: %d", analogRead(CYD_LDR_PIN));
  for (uint8_t i = 0; i < oversample_count; i++)
  {
    raw_accumulator += analogRead(CYD_LDR_PIN);
  }

  /* Result is 12-bit (0-4095) */
  uint16_t ldr_raw = (uint16_t)(raw_accumulator / oversample_count);

  /* --- Goal 1: Send CAN Message (0x50F) --- */
  uint8_t data[DATA_ADC_RAW_DLC];
  /* Use your global myNodeID array */
  memcpy(&data[0], (const void *)myNodeID, 4);

  data[4] = ldr_submod_idx;            /* Sensor ID / Submodule index */
  data[5] = (uint8_t)(ldr_raw >> 8);   /* High byte of 12-bit ADC */
  data[6] = (uint8_t)(ldr_raw & 0xFF); /* Low byte of 12-bit ADC */

  /* Reusing your existing TWAI routine */

  canEnqueueMessage(DATA_ADC_RAW_ID, data, DATA_ADC_RAW_DLC);
#endif
}

static void internalSubmoduleTick(uint32_t now)
{
  uint8_t list[MAX_SUB_MODULES];
  uint8_t n = nodeGetInternalPeriodicProducers(list, sizeof(list));

  ESP_LOGV(TAG, "[PRODUCER] Checking %u internal events", n);

  for (uint8_t i = 0; i < n; i++)
  {
    subModule_t *sub = nodeGetSubModule(list[i]);
    ESP_LOGV(TAG, "[PRODUCER] Checking internal event for sub %u interval %u", list[i], sub->producer_period_ms);

    if (now - sub->runTime.last_published_value >= sub->producer_period_ms)
    {
      enqueueInputCmd(INPUT_CMD_INTERNAL_EVENT, list[i], now);
      sub->runTime.last_published_value = now;

      ESP_LOGV(TAG, "[PRODUCER] Enqueueing internal event for sub %u interval %u", list[i], sub->producer_period_ms);
    }
  }
}

/**
 * @brief Manages periodic transmissions in TaskTWAI
 */
static void managePeriodicMessages()
{
  static uint32_t lastInternalsTick = 0;
  static uint32_t lastIntro = 0;
  static uint32_t lastCanHealth = 0;

  uint32_t currentMillis = millis();

  if (currentMillis - lastInternalsTick >= 1000)
  {
    lastInternalsTick = currentMillis;
    internalSubmoduleTick(currentMillis);
    ESP_LOGV(TAG, "[PRODUCER] Scanning internal submodules");
  }

  /** Introduction as Heartbeat - Every 10 Seconds
      We send this as the heartbeat to save bandwidth,
      unless FLAG_SEND_INTRODUCTION is manually set by a Master request. */
  if (currentMillis - lastIntro >= 10000)
  {
    lastIntro = currentMillis;
    ESP_LOGI(TAG, "[PRODUCER] Sending heartbeat");
    sendIntroduction(0); /* send the node introduction message as heartbeat */
    // TODO: readCydLdr should be configured as a producer
    readCydLdr(); /* Read CYD LDR and send that data to the bus */
    static UBaseType_t highWater = 0;

    UBaseType_t waiting = uxQueueMessagesWaiting(canMsgRxQueue);
    UBaseType_t free = uxQueueSpacesAvailable(canMsgRxQueue);

    if (waiting > highWater)
      highWater = waiting;

    ESP_LOGI(TAG,
             "[PRODUCER] RX Queue: waiting=%lu free=%lu highWater=%lu",
             (unsigned long)waiting,
             (unsigned long)free,
             (unsigned long)highWater);

    /* If we are stuck mid-sequence, reset after 10s of silence */
    void introResetSequence(void);
  }

  if (currentMillis - lastCanHealth >= 60000)
  {
    lastCanHealth = currentMillis;
    canHealth_t *h = twaiGetCanHealth();

    canHealthLog(h); /* Log CAN Health every minute */
  }

  /** Run submodule data collection */
  updateSubModules();
}

/**
 * @brief Initializes GPIO ISR handlers for active input submodules.
 *
 * This function is responsible for attaching the ISR handler to each active
 * input submodule. It iterates over all submodules and checks if the submodule
 * is active in producer logic and if it is an input submodule. If both
 * conditions are true, the ISR handler is attached to the corresponding GPIO pin.
 *
 * @note This function is called once during initialization and should not be called
 * again.
 */
void attachGpioIsrInit(void)
{
  for (int i = 0; i < MAX_SUB_MODULES; i++)
  {
    subModule_t *sub = nodeGetActiveSubModule(i);

    if (!sub)
      continue;

    /* Only input submodules can have ISRs */
    if (!nodeIsInputSubmodule(i))
      continue;

    /* Submodule must be active in producer logic */
    if (!(sub->producer_flags & PRODUCER_FLAG_ACTIVE))
      continue;

    /* Validate invariant: personalityIndex must equal submodule index */
    if (sub->personalityIndex != i)
    {
      ESP_LOGE("NODE", "Invalid personalityIndex: sub %d has %d",
               i, sub->personalityIndex);
      continue;
    }

    const personalityDef_t *p = nodeGetActivePersonality(i);
    if (!p)
    {
      ESP_LOGW(TAG, "Referenced personality is inactive or invalid %d", i);
      continue;
    }

    const uint8_t pin = p->gpioPin;
    if (pin == 0) /* GPIO0 reserved on ESP32 */
      continue;

    /* Attach ISR */
    attachDigitalInputISR(pin, i);
  }
}

static void handleInputCommand(inputCommandMsg_t *cmd)
{
  const uint8_t subIdx = cmd->index;

  ESP_LOGV(TAG, "\n[PRODUCER] Input command received: %u (sub=%u)\n", (uint8_t)cmd->type, subIdx);

  if (!nodeIsActiveSubmodule(subIdx))
    return;

  switch (cmd->type)
  {
  case INPUT_CMD_INTERNAL_EVENT:
  case INPUT_CMD_GPIO_EVENT:
  {
    producer_event_t evt =
        producerHandleGpioEvent(subIdx, cmd->timestamp);

    if (evt.error)
    {
      ESP_LOGW(TAG, "[PRODUCER] GPIO event handler returned an error, aborting producer processing.");
      break;
    }
    else if (evt.ready)
    {
      producerPublishEvent(&evt);
    }
    else
    { /* log a debug message if the event was processed but not ready to publish */
      ESP_LOGD(TAG, "[PRODUCER] GPIO event ignored: not ready (sub=%u)", subIdx);
    }
    break;
  }

  case INPUT_CMD_CFG_CHANGE:
    // future: reload producer flags, periods, etc.
    break;

  case INPUT_CMD_DEBOUNCE_TICK:
    // probably unused now
    break;

  default:
    ESP_LOGW(TAG, "Unknown input event: %d", cmd->type);
    break;
  }
}

static void TaskProducer(void *pvParameters)
{

  /* Install ISR service */
  initGpioIsrService();

  ESP_LOGI(TAG, "[RTOS] Producer task starting...");

  /* Block until TWAI driver is installed */
  while (!twaiIsDriverInstalled())
  {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  ESP_LOGI(TAG, "[RTOS] Producer task started.");

  /* Initialize GPIO ISR handlers */
  attachGpioIsrInit();

  /* Last time periodic messages were processed */
  uint32_t lastPeriodicMs = millis();

  for (;;)
  {
    inputCommandMsg_t cmd; /**< Input command message */

    /* Wait up to 10 ms for an input command */
    if (xQueueReceive(inputTaskQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      /* Handle input command (GPIO event, cfg change, internal event, etc.) */
      handleInputCommand(&cmd); /**< To be implemented separately */
    }

    /* Periodic producer work (~100 Hz) */
    const uint32_t now = millis();
    if ((now - lastPeriodicMs) >= 10U)
    {
      managePeriodicMessages(); /**< Existing periodic CAN producer */
      lastPeriodicMs = now;
    }
  }
}

/* =========================================================================
 *  Public Functions
 * ========================================================================= */

void startTaskProducer(void)
{
  xTaskCreate(
      TaskProducer,
      "Task Producer",
      TASK_PRODUCER_STACK_SIZE,
      NULL,
      tskLowPriority,
      &xProducerHandle);
}
