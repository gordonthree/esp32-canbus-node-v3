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

static const char *TAG = "task_twai";

static bool can_driver_installed = false;
static bool can_suspended = false;
static unsigned long lastCanError = 0;
static uint8_t fullResetCount = 0;
static bool recoveryInProgress = false;
static uint32_t lastHealthUpdate = 0;

const uint32_t LOG_INTERVAL = 2000;      /* Log once every 2 seconds max */
const uint32_t PASS_INTERVAL = 30000;    /* Attempt recovery after 30 seconds */
const uint32_t RECOVERY_INTERVAL = 250;  /* Allow 250 ms for recovery */
const uint8_t MAX_RECOVERY_ATTEMPTS = 5; /* Allow 5 recovery attempts */

static canHealth_t canHealth = {0};

/* =========================================================================
  Private Types
  ========================================================================= */

typedef struct
{
  uint32_t dropCount;
  uint32_t timestamp;
} dropCount_t;

static dropCount_t rxDropCount = {0, 0};
static dropCount_t txDropCount = {0, 0};

typedef struct
{
  uint32_t lastErrPassLog;
  uint32_t firstPassLog;
  uint32_t lastBusErrLog;
  uint32_t lastTxFailLog;
  uint32_t lastRxFullLog;
  uint32_t lastBusOffLog;
  uint32_t lastRxDropLog;
  uint32_t lastTxDropLog;
  uint32_t recoveryTimer;
  uint32_t dropCountRx;
  uint32_t dropCountTx;
  uint8_t fullResetCount;
  bool recoveryInProgress;
} twai_diag_t;

static twai_diag_t twaiDiag = {0};

typedef enum
{
  TWAI_ACTION_NONE = 0,
  TWAI_ACTION_HARD_RESET,
  TWAI_ACTION_SHUTDOWN
} twai_action_t;

/* ==========================================================================
  Private Declarations
  ========================================================================= */

/* Initialize the TWAI driver */
static void twaiInit(void);

/* TWAI task */
static void TaskTWAI(void *pvParameters);

/* TWAI error recovery actions */
static void twaiPerformHardReset(void);
static void twaiPerformShutdown(void);

/* Handle TWAI driver alerts */
static twai_action_t twaiHandleAlerts(uint32_t alerts, const twai_status_info_t *st);

/* Utility functions */
static inline void twaiTxOffsetDelay(void);
static void canHealthUpdateDriverStats(canHealth_t *h);

/* =========================================================================
  Public Variables
  ========================================================================= */

/* =========================================================================
  Private Functions
  ========================================================================= */

static inline void twaiTxOffsetDelay(void)
{
  // 5–30 µs random delay for 500 kbps bus
  uint32_t us = 5 + (esp_random() % 25);
  ets_delay_us(us);
}

static twai_action_t twaiHandleAlerts(uint32_t alerts, const twai_status_info_t *st)
{
  uint32_t now = millis();
  canHealth_t *health = &canHealth;

  if (alerts & TWAI_ALERT_ARB_LOST)
  {
    health->arb_lost_count++;
    health->last_event_ms = now;
  }

  // ERROR PASSIVE
  if (alerts & TWAI_ALERT_ERR_PASS)
  {
    if (twaiDiag.firstPassLog == 0)
    {
      twaiDiag.firstPassLog = now;
    }
    if (now - twaiDiag.lastErrPassLog > LOG_INTERVAL)
    {
      ESP_LOGW(TAG, "[TWAI] Alert: Controller has become error passive.");
      twaiDiag.lastErrPassLog = now;
    }
    health->error_passive_events++;
    health->last_event_ms = now;
  }

  // Recovery timeout → hard reset or shutdown
  if (twaiDiag.recoveryInProgress && (now - twaiDiag.recoveryTimer > RECOVERY_INTERVAL))
  {
    twaiDiag.recoveryInProgress = false;
    twaiDiag.fullResetCount++;
    health->recovery_resets++;
    health->last_event_ms = now;
    // if (twaiDiag.fullResetCount >= MAX_RECOVERY_ATTEMPTS)
    // {
    //   ESP_LOGE(TAG, "[TWAI] Too many recovery attempts, stopping TWAI driver. Reboot required.");
    //   return TWAI_ACTION_SHUTDOWN;
    // }
    ESP_LOGE(TAG, "[TWAI] Recovery timer expired, attempting TWAI hard reset");
    return TWAI_ACTION_HARD_RESET;
  }

  // BUS ERROR
  if (alerts & TWAI_ALERT_BUS_ERROR)
  {
    health->bus_error_count++;
    health->last_event_ms = now;
    if (now - twaiDiag.lastBusErrLog > LOG_INTERVAL)
    {
      ESP_LOGW(TAG, "[TWAI] Alert: Bus error. Count: %d", st->bus_error_count);
      twaiDiag.lastBusErrLog = now;
    }
    if (st->bus_error_count > TWAI_BUS_ERR_RECOVERY &&
        !twaiDiag.recoveryInProgress)
    {
      ESP_LOGE(TAG, "[TWAI] Too many bus errors, attempting TWAI hard reset");
      twaiDiag.recoveryTimer = now;
      twaiDiag.recoveryInProgress = true;
      return TWAI_ACTION_HARD_RESET;
    }
  }

  // TX FAILED
  if (alerts & TWAI_ALERT_TX_FAILED)
  {
    health->tx_failed_count++;
    health->last_event_ms = now;
    twaiDiag.dropCountTx++;
    if (now - twaiDiag.lastTxFailLog > LOG_INTERVAL)
    {
      ESP_LOGE(TAG, "[TWAI] Alert: Transmission failed.");
      twaiDiag.lastTxFailLog = now;
    }
  }

  // RX QUEUE FULL
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
  {
    health->rx_queue_full_events++;
    health->last_event_ms = now;
    if (now - twaiDiag.lastRxFullLog > LOG_INTERVAL)
    {
      ESP_LOGE(TAG, "[TWAI] Alert: RX queue full.");
      twaiDiag.lastRxFullLog = now;
    }
    twai_message_t msg; /* drain rx queue */
    while (twai_receive(&msg, 0) == ESP_OK)
    { /* discard */
    }
  }

  // BUS OFF
  if (alerts & TWAI_ALERT_BUS_OFF)
  {
    health->bus_off_events++;
    health->last_event_ms = now;
    if (now - twaiDiag.lastBusOffLog > LOG_INTERVAL)
    {
      ESP_LOGE(TAG, "[TWAI] BUS OFF, scheduling recovery.");
      twaiDiag.lastBusOffLog = now;
      twaiDiag.recoveryTimer = now;
      twaiDiag.recoveryInProgress = true;
    }
  }

  // BUS RECOVERED
  if (alerts & TWAI_ALERT_BUS_RECOVERED)
  {
    health->bus_recovered_events++;
    health->last_event_ms = now;
    ESP_LOGD(TAG, "[TWAI] Bus recovered, resuming normal operation");
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK)
    { /* discard */
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    twaiDiag.dropCountRx = 0;
    twaiDiag.dropCountTx = 0;
    twaiDiag.firstPassLog = 0;
    twaiDiag.fullResetCount = 0;
    twaiDiag.recoveryInProgress = false;
  }

  // Drop reporting
  if (twaiDiag.dropCountRx > 0 && now - twaiDiag.lastRxDropLog > LOG_INTERVAL)
  {
    ESP_LOGE(TAG, "[TWAI] RX queue dropped %u messages since the last report.", twaiDiag.dropCountRx);
    twaiDiag.lastRxDropLog = now;
    rxDropCount.dropCount = twaiDiag.dropCountRx;
    rxDropCount.timestamp = now;
    twaiDiag.dropCountRx = 0;
  }

  if (twaiDiag.dropCountTx > 0 && now - twaiDiag.lastTxDropLog > LOG_INTERVAL)
  {
    ESP_LOGE(TAG, "[TWAI] TX queue dropped %u messages since the last report.", twaiDiag.dropCountTx);
    twaiDiag.lastTxDropLog = now;
    txDropCount.dropCount = twaiDiag.dropCountTx;
    txDropCount.timestamp = now;
    twaiDiag.dropCountTx = 0;
  }

  return TWAI_ACTION_NONE;
}

static void twaiInit(void)
{
  bool initSuccess = true;

  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);

  // twai_general_config_t g_config =
  //     TWAI_GENERAL_CONFIG_DEFAULT(
  //         (gpio_num_t)TX_PIN,
  //         (gpio_num_t)RX_PIN,
  //         TWAI_MODE_NORMAL);

  // // Override only what you need
  // g_config.tx_queue_len = TWAI_TX_QUEUE_LEN;
  // g_config.rx_queue_len = TWAI_RX_QUEUE_LEN;
  // g_config.alerts_enabled = TWAI_ALERT_ALL; // required for install
  // g_config.intr_flags = ESP_INTR_FLAG_IRAM;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); /* accept all messages, filter in software */

  /* Install TWAI driver */
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    ESP_LOGI(TAG, "[TWAI] Driver installed.");
  }
  else
  {
    initSuccess = false;
    ESP_LOGE(TAG, "[TWAI] Failed to install driver, reboot recommended.");
    // vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Start TWAI driver */
  if (twai_start() == ESP_OK)
  {
    ESP_LOGI(TAG, "[TWAI] Driver started.");
  }
  else
  {
    initSuccess = false;
    ESP_LOGE(TAG, "[TWAI] Failed to start driver, reboot recommended.");
    // vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states */
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_RX_QUEUE_FULL |
                              TWAI_ALERT_TX_IDLE |
                              TWAI_ALERT_TX_SUCCESS |
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_BUS_ERROR |
                              TWAI_ALERT_ARB_LOST |
                              TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED;

  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    ESP_LOGI(TAG, "[TWAI] Alerts reconfigured.");
  }
  else
  {
    initSuccess = false;
    ESP_LOGW(TAG, "[TWAI] Failed to reconfigure alerts.");
    // vTaskDelete(NULL); /* <--- Safety fix */
  }

  /* TWAI driver is now successfully installed and started */
  can_driver_installed = initSuccess;
}

static void twaiPerformShutdown(void)
{
  ESP_LOGE(TAG, "[TWAI] Entering TWAI shutdown mode");

  twai_stop();
  twai_driver_uninstall();

  can_driver_installed = false;
  can_suspended = true;

  xQueueReset(canMsgRxQueue);
  xQueueReset(canMsgTxQueue);

  lastCanError = millis();

  // Optional: small delay to stabilize
  vTaskDelay(pdMS_TO_TICKS(50));
}

static void twaiPerformHardReset(void)
{
  ESP_LOGE(TAG, "[TWAI] Performing hard reset of TWAI driver");

  twai_stop();
  twai_driver_uninstall();

  vTaskDelay(pdMS_TO_TICKS(10)); // allow hardware to settle

  can_driver_installed = false;

  twaiInit(); // reinstall + restart + reconfigure alerts

  vTaskDelay(pdMS_TO_TICKS(10)); // allow bus to settle
}

static void canHealthUpdateDriverStats(canHealth_t *h)
{
  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK)
  {
    h->tx_error_count = s.tx_error_counter;
    h->rx_error_count = s.rx_error_counter;
    h->tx_failed_count = s.tx_failed_count;
    h->rx_missed_count = s.rx_missed_count;
    h->rx_overrun_count = s.rx_overrun_count;
    h->arb_lost_count = s.arb_lost_count;
    h->bus_error_count = s.bus_error_count;
    h->controller_state = s.state;
  }
}

static void TaskTWAI(void *pvParameters)
{
  int loopCount = 0;
  uint32_t now = millis();

  /* give some time at boot for the cpu setup other parameters */
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "[RTOS] TWAI task started.");

  /* Initialize the TWAI driver */
  twaiInit();

  /* Begin main can bus RX/TX loop */
  for (;;)
  {
    if (!can_driver_installed || can_suspended)
    {
      /* Driver not installed or app has requested the bus be suspended */
      xQueueReset(canMsgRxQueue);     /* Clear the RX queue */
      xQueueReset(canMsgTxQueue);     /* Clear the TX queue */
      vTaskDelay(pdMS_TO_TICKS(100)); /* Idle the task */
      continue;                       /* Skip the rest of the loop */
    }
    /* Check if alert happened */
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));

    /* Read TWAI status */
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    twai_action_t action = twaiHandleAlerts(alerts_triggered, &twaistatus);

    if (action == TWAI_ACTION_SHUTDOWN)
    {
      twaiPerformShutdown();
      continue;
    }

    if (action == TWAI_ACTION_HARD_RESET)
    {
      twaiPerformHardReset();
      continue;
    }

    /* Update canHealth_t */
    if (now - lastHealthUpdate >= 1000)
    {
      canHealthUpdateDriverStats(&canHealth);
      lastHealthUpdate = now;
    }

    /* --- Load RX queue --- */
    // if (alerts_triggered & TWAI_ALERT_RX_DATA)
    if (true)
    {
      /* One or more messages received. Handle all. */
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK)
      {
        can_msg_t rx = toCanMsg(&message);                           /* convert to can_msg_t */
        if (xQueueSend(canMsgRxQueue, &rx, QUEUE_NO_WAIT) != pdTRUE) /* Attempt to add message to queue */
        {                                                            /* Queue full, discard message */
          twaiDiag.dropCountRx++;
        }
      }
    }

    /* --- Drain TX queue --- */

    twai_message_t tx = {0}; /* Zero-initialize new message */

    /* Retrieve message from queue */
    bool can_tx = (twaistatus.state == TWAI_STATE_RUNNING);
    if (can_tx)
    {
      while (xQueueReceive(canMsgTxQueue, &tx, 0) == pdTRUE)
      {
        twaiTxOffsetDelay(); /* microsecond delay between frames */
        /* Transmit message, no delay */
        esp_err_t res = twai_transmit(&tx, 0);
        if (res != ESP_OK)
        {
          /* retry once */
          vTaskDelay(pdMS_TO_TICKS(1));
          res = twai_transmit(&tx, 0);
          if (res != ESP_OK)
          {
            /* still not working, discard message */
            twaiDiag.dropCountTx++;
          }
        }
      }
    }

    // vTaskDelay(pdMS_TO_TICKS(1)); /* minimal task delay */
    vTaskDelay(0); // yield only
  }
}

/* =========================================================================
  Public API Functions
  ========================================================================= */
bool twaiIsDriverInstalled()
{
  return can_driver_installed;
}

bool twaiIsSuspended()
{
  return can_suspended;
}

void twaiSetSuspended(bool suspended)
{
  can_suspended = suspended;
}

unsigned long twaiLastErrorTime()
{
  return lastCanError;
}

void canEnqueueMessage(uint16_t msgid, const uint8_t *data, uint8_t dlc)
{
  if (dlc > CAN_MAX_DLC)
    dlc = CAN_MAX_DLC; /* Safety check */

  can_msg_t frame;
  frame.identifier = msgid;
  frame.data_length_code = dlc;
  memcpy(frame.data, data, dlc);

  /* print debug info */
  // ESP_LOGV(TAG, "[TWAI] Enqueue ID=0x%03X DLC=%d", msgid, dlc);

  /* loop through *data and print it */
  // for (int i = 0; i < dlc; i++) {
  //   ESP_LOGV(TAG, "[TWAI] Data=0x%02X ", data[i]);
  // }

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
    const uint8_t dlc)
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

void startTaskTWAI(void)
{
  /* Start the CAN task */
  xTaskCreate(
      TaskTWAI,             /* Task function */
      "Task TWAI",          /* name of task */
      TASK_TWAI_STACK_SIZE, /* Stack size of task */
      NULL,                 /* parameter of the task */
      tskHigherPriority,    /* priority of the task */
      &xTWAIHandle          /* Task handle to keep track of created task */
  );
}

canHealth_t *twaiGetCanHealth()
{
  return &canHealth;
}