#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "task_twai.h"
#include "task_producer.h"
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
static void handleProducerTick(uint32_t now);
static void readCydLdr();
static void managePeriodicMessages();
static void TaskProducer(void *pvParameters);

/* =========================================================================
 *  Private functions
 * ========================================================================= */

static void updateSubModules()
{
  for (uint8_t i = 0; i < nodeGetInfo()->subModCnt; i++)
  {
    subModule_t *sub = nodeGetSubModule(i);
    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    if (!p)
    {
      ESP_LOGW(TAG, "[PRODUCER] Error: Personality lookup failed for index %u", i);
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
      // TODO: Figure out how to read NeoPixelBus status?
      break;

    /* Analog RGBW output */
    case PERS_RGBW_OUTPUT:
      // TODO
      break;

    /* Analog DAC output */
    case PERS_ANA_OUTPUT:
      // TODO: Read DAC output value?
      break;

    /* Digital GPIO personalities */
    case PERS_GPIO_OUTPUT:
    case PERS_GPIO_INPUT:
      // sub->runTime.valueU32 = digitalRead(myPin); /**< Capture digital value */
      break;

      /* Analog ADC input */
    case PERS_ANALOG_INPUT:
      sub->runTime.valueU32 = analogRead(myPin); /**< Capture analog value */
      break;

    case VIRT_FREE_HEAP:
      value = (uint32_t)xPortGetFreeHeapSize();
      break;

    case VIRT_WIFI_RSSI:
    {
      int rssi = wifiGetRssi(); /* get RSSI */
      if (rssi < 0)
        rssi = -rssi; /* absolute value */
      value = (uint32_t)rssi;
      break;
    }

    case VIRT_RTOS_HIGHWATERMARK:
      value = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
      break;

    case VIRT_INTERNAL_TEMPERATURE:
      value = (uint32_t)temperatureRead();
      break;

    case VIRT_VREF_VOLTAGE:
      // value = (uint32_t)readInternalVref();
      value = 1024; // TODO: implement actual function
      break;

    default:
      ESP_LOGW(TAG, "[PRODUCER] Error: Unknown personality %u", sub->personalityId);
      // Unknown virtual personality — ignore safely
      continue;
    }

    // sub->runTime.valueU32       = value;     /* update value */
    // sub->runTime.last_change_ms = millis();  /* update timestamp */
  } /* end of for loop */
}

static void handleProducerTick(uint32_t now)
{
  /** Check for producer event */
  producer_event_t evt = producerTick(now);

  if (evt.error)
  {
    ESP_LOGW(TAG, "[PRODUCER] ProducerTick returned error, aborting producer processing");
    return;
  }
  if (!evt.ready)
    return;

  if (evt.sub_idx >= nodeGetInfo()->subModCnt)
  {
    ESP_LOGW(TAG, "[PRODUCER] Error: bad sub_idx %u (max %u)",
             evt.sub_idx, nodeGetInfo()->subModCnt - 1);
    return;
  }

  /** Lookup personality */
  const nodeInfo_t &node = *nodeGetInfo();
  const subModule_t &sub = *nodeGetSubModule(evt.sub_idx);
  const personalityDef_t *p = &runtimePersonalityTable[nodeGetSubModule(evt.sub_idx)->personalityIndex];
  if (!p)
  {
    ESP_LOGW(TAG, "[PRODUCER] Error: personality lookup failed for sub %u", evt.sub_idx);
    return;
  }

  uint8_t dlc = p->dataMsgDlc;
  if (dlc < CAN_DATAMSG_MIN_DLC)
    dlc = CAN_DATAMSG_MIN_DLC; /**< enforce minimum DLC for producer messages 6-bytes */
  if (dlc > CAN_MAX_DLC)
    dlc = CAN_MAX_DLC; /**< enforce maximum DLC */

  uint8_t payload[CAN_MAX_DLC] = {0}; /**< zero-initialize */

  /* Node ID (big-endian) */
  payload[0] = (node.nodeID >> BYTE_SHIFT3) & BYTE_MASK;
  payload[1] = (node.nodeID >> BYTE_SHIFT2) & BYTE_MASK;
  payload[2] = (node.nodeID >> BYTE_SHIFT) & BYTE_MASK;
  payload[3] = (node.nodeID) & BYTE_MASK;

  /* Submodule index */
  payload[4] = evt.sub_idx;

  const uint32_t valueU32 = evt.value;

  if (valueU32 <= 0xFF)
  { /* Single byte value, send it in position 5 */
    payload[5] = valueU32 & BYTE_MASK;
  }
  else if (valueU32 <= 0xFFFF)
  { /* Two-byte value */
    payload[5] = (valueU32 >> BYTE_SHIFT) & BYTE_MASK;
    payload[6] = (valueU32)&BYTE_MASK;
  }
  else
  { /* Three-byte value */
    payload[5] = (valueU32 >> BYTE_SHIFT2) & BYTE_MASK;
    payload[6] = (valueU32 >> BYTE_SHIFT) & BYTE_MASK;
    payload[7] = (valueU32)&BYTE_MASK;
  }

  /* Send message */
  canEnqueueMessage(p->dataMsgId, payload, dlc);

  ESP_LOGI(TAG, "[PRODUCER] Tx: sub %u msg 0x%03X dlc %u val %u",
           evt.sub_idx, p->dataMsgId, dlc, evt.value);
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

/**
 * @brief Manages periodic transmissions in TaskTWAI
 */
static void managePeriodicMessages()
{
  static uint32_t lastHeartbeat = 0;
  static uint32_t lastIntro = 0;
  uint32_t currentMillis = millis();

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

    /* If we are stuck mid-sequence, reset after 10s of silence */
    void introResetSequence(void);
  }

  /** Update physical submodules */
  updateSubModules();

  /** Call the producer tick */
  handleProducerTick(currentMillis);
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
  /* Skip undefined submodules */
  const int count = nodeGetSubModuleCount();

  for (int i = 0; i < count; i++)
  {

    const subModule_t *sub = nodeGetSubModule(i);
    if (!sub)
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

    const personalityDef_t *p = nodeGetPersonality(i);
    if (!p)
      continue;

    const uint8_t pin = p->gpioPin;
    if (pin == 0) /* GPIO0 reserved on ESP32 */
      continue;

    /* Only attach ISR for input submodules */
    if (nodeIsInputSubmodule(i))
    {
      attachDigitalInputISR(pin, i);
    }
  }
}

static void TaskProducer(void *pvParameters)
{
  ESP_LOGI(TAG, "[RTOS] Producer task starting...");

  /* Block until TWAI driver is installed */
  while (!twaiIsDriverInstalled())
  {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  ESP_LOGI(TAG, "[RTOS] Producer task started.");

  attachGpioIsrInit(); /* Initialize GPIO ISR handlers */

  for (;;)
  {
    managePeriodicMessages();      /* Publish periodic CAN messages */
    vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz producer tick */
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
