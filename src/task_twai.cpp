#include <Arduino.h>

#include "driver/twai.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "can_platform.h"
#include "task_twai.h"
#include "freertos.h"

#include "esp_log.h"

/* ========================================================================= 
  Private Constants and Variables
  ========================================================================= */

static const char* TAG = "task_twai";


static bool can_driver_installed = false;
static bool can_suspended = false;
static unsigned long lastCanError = 0;

/* ========================================================================= 
  Public Variables
  ========================================================================= */




/* ========================================================================= 
  Private Functions
  ========================================================================= */
static void twaiInit(void) 
{
  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); /* accept all messages, filter in software */

  /* Install TWAI driver */
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    ESP_LOGI(TAG, "[TWAI] Driver installed.");
  } else {
    ESP_LOGE(TAG, "[TWAI] Failed to install driver, reboot recommended.");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Start TWAI driver */
  if (twai_start() == ESP_OK) {
    ESP_LOGI(TAG, "[TWAI] Driver started.");
  } else {
    ESP_LOGE(TAG, "[TWAI] Failed to start driver, reboot recommended.");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states */
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    ESP_LOGI(TAG, "[TWAI] Alerts reconfigured.");
  } else {
    ESP_LOGW(TAG, "[TWAI] Failed to reconfigure alerts.");
    vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* TWAI driver is now successfully installed and started */
  can_driver_installed = true;   
}

void TaskTWAI(void *pvParameters)
{
  int loopCount = 0;
  
  /* give some time at boot for the cpu setup other parameters */
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "[RTOS] TWAI task started.");

  /* Initialize the TWAI driver */
  twaiInit();

  /* Begin main can bus RX/TX loop */
  for (;;) {
    if (!can_driver_installed || can_suspended) {
      /* Driver not installed or bus suspended */
      vTaskDelay(pdMS_TO_TICKS(100)); /* Idle the task */
      continue; /* Skip the rest of the loop */
    }
    /* Check if alert happened */
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    /* Timestamps for throttling Serial output (static to persist across loops) */
    static uint32_t lastErrPassLog = 0;
    static uint32_t lastBusErrLog = 0;
    static uint32_t lastTxFailLog = 0;
    static uint32_t lastRxFullLog = 0;
    const  uint32_t LOG_INTERVAL  = 2000; /* Log once every 2 seconds max */

    /* Handle alerts */
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      if (millis() - lastErrPassLog > LOG_INTERVAL) {
        ESP_LOGW(TAG, "[TWAI] Alert: Controller has become error passive.");
        lastErrPassLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      if (millis() - lastBusErrLog > LOG_INTERVAL) {
        ESP_LOGW(TAG, "[TWAI] Alert: Bus error. Count: %d", twaistatus.bus_error_count);
        lastBusErrLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      if (millis() - lastTxFailLog > LOG_INTERVAL) {
        ESP_LOGW(TAG, "[TWAI] Alert: Transmission failed.");
        lastTxFailLog = millis();
      }
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      if (millis() - lastRxFullLog > LOG_INTERVAL) {
        ESP_LOGW(TAG, "[TWAI] Alert: RX queue full.");
        lastRxFullLog = millis();
      }
    }

    /* --- Load RX queue --- */
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      /* One or more messages received. Handle all. */
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        can_msg_t rx = toCanMsg(&message);
        xQueueSend(canMsgRxQueue, &rx, QUEUE_NO_WAIT);
      }
    }

    /* --- Drain TX queue --- */
    twai_message_t tx = {0}; /* Zero-initialize new message */
    while (xQueueReceive(canMsgTxQueue, &tx, 0) == pdTRUE) {
        esp_err_t res = twai_transmit(&tx, pdMS_TO_TICKS(10)); /* Transmit message */
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "[TWAI] TX failed (%d) ID=0x%03X", res, tx.identifier);
        } else {
            ESP_LOGI(TAG, "[TWAI] TX OK ID=0x%03X", tx.identifier);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1)); /* minimal task delay */
  }
}


/* ========================================================================= 
  Public API Functions
  ========================================================================= */
bool twaiIsDriverInstalled() {
  return can_driver_installed;
}

bool twaiIsSuspended() {
  return can_suspended;
}

void twaiSetSuspended(bool suspended) {
  can_suspended = suspended;
}

unsigned long twaiLastErrorTime() {
  return lastCanError;
}

void canEnqueueMessage(uint16_t msgid, const uint8_t *data, uint8_t dlc)
{
    if (dlc > CAN_MAX_DLC) dlc = CAN_MAX_DLC;   /* Safety check */

    can_msg_t frame;
    frame.identifier       = msgid;
    frame.data_length_code = dlc;
    memcpy(frame.data, data, dlc);

    /* print debug info */
    ESP_LOGV(TAG, "[TWAI] Enqueue ID=0x%03X DLC=%d", msgid, dlc);

    /* loop through *data and print it */
    for (int i = 0; i < dlc; i++) {
      ESP_LOGV(TAG, "[TWAI] Data=0x%02X ", data[i]);
    }

    CanTxMsg_t hw = toTwaiMsg(&frame);
    xQueueSend(canMsgTxQueue, &hw, QUEUE_NO_WAIT);
}

/**
 * @brief Send a uint32_t on the CAN bus
 * @param nodeID The node ID to use for transmission
 * @param bigNumber The uint32_t to be sent
 * @param canMsgId The message ID to use for transmission (default: DATA_EPOCH_ID)
 * @param dlc The data length code to use for transmission (default: DATA_EPOCH_DLC)
 *
 * This function takes a uint32_t and sends it on the CAN bus
 * with the given message ID and data length code.
 */
void canSendUint32(
    const uint32_t nodeID, 
    const uint32_t bigNumber, 
    const uint32_t canMsgId, 
    const uint8_t dlc
  ) 
{
  uint8_t dataBytes[dlc];

  dataBytes[0] = (nodeID >> 24) & 0xFF;
  dataBytes[1] = (nodeID >> 16) & 0xFF;
  dataBytes[2] = (nodeID >> 8) & 0xFF;
  dataBytes[3] = (nodeID & 0xFF);
  dataBytes[4] = (bigNumber >> 24) & 0xFF;
  dataBytes[5] = (bigNumber >> 16) & 0xFF;
  dataBytes[6] = (bigNumber >> 8) & 0xFF;
  dataBytes[7] = (bigNumber & 0xFF);

  canEnqueueMessage(canMsgId, (uint8_t *)dataBytes, dlc);
}

void startTaskTWAI(void) {
  /* Start the CAN task */
  xTaskCreate(
    TaskTWAI,              /* Task function */
    "Task TWAI",           /* name of task */
    TASK_TWAI_STACK_SIZE,  /* Stack size of task */
    NULL,                  /* parameter of the task */
    tskNormalPriority,     /* priority of the task */
    &xTWAIHandle           /* Task handle to keep track of created task */
  );
}