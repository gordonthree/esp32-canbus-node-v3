#pragma once
#include "driver/twai.h"

#include "canbus_project.h"    /**< my various CAN functions and structs */
#include "freertos.h"          /*<< RTOS task handle declarations */

#define TWAI_RX_QUEUE_LEN       (15U)
#define TWAI_TX_QUEUE_LEN       (15U)
  
#define TWAI_BUS_ERR_RECOVERY   (100U)

/* ========================================================================= 
   Public types 
   ========================================================================= */

typedef struct {
    // Raw TWAI driver status (mirrors twai_status_info_t)
    int tx_error_count;          // TEC
    int rx_error_count;          // REC
    int tx_failed_count;         // TWAI_STATUS_TX_FAILED_COUNT
    int rx_missed_count;         // TWAI_STATUS_RX_MISSED_COUNT
    int rx_overrun_count;        // TWAI_STATUS_RX_OVERRUN_COUNT
    int arb_lost_count;          // TWAI_STATUS_ARB_LOST_COUNT
    int bus_error_count;         // TWAI_STATUS_BUS_ERROR_COUNT
    twai_state_t controller_state;

    // High‑level events you care about
    uint32_t bus_off_events;     // how many times BUS-OFF occurred
    uint32_t error_passive_events;
    uint32_t recovery_resets;    // how many hard resets performed
    uint32_t rx_queue_full_events;
    uint32_t tx_queue_dropped_events;
    uint32_t bus_recovered_events;

    // Timing
    uint32_t last_event_ms;      // last time anything changed
} canHealth_t;


/* ========================================================================= 
  FreeRTOS TWAI Task Public API Functions
  ========================================================================= */

/* expose task to start the TWAI task */
void startTaskTWAI();

/* Getters to on TWAI state */

bool twaiIsDriverInstalled();
unsigned long twaiLastErrorTime();

canHealth_t *twaiGetCanHealth();

bool twaiIsSuspended();
void twaiSetSuspended(bool suspended);

void canEnqueueMessage(
    uint16_t msgid, 
    const uint8_t *data, 
    uint8_t dlc
  );
  
void canSendUint32(
    const uint32_t nodeID, 
    const uint32_t bigNumber, 
    const uint32_t canMsgId, 
    const uint8_t dlc
  );