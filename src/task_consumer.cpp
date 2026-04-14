/**
 * @file task_consumer.cpp
 * @brief CAN consumer logic
 *
 */

/* === Framework includes === */
#include <Arduino.h>

/* === Standard library includes === */
#include <stddef.h>
#include <stdio.h>

/* === Local includes === */
#include "can_producer.h" /**< CAN producer routines and constants */
#include "can_router.h"   /**< CAN routing (subcriber) routines and constants */
#include "node_state.h"   /**< Node and sub-module state table */
#include "personality_table.h" /**< Node and sub-module personality table */
#include "submodule_types.h"   /**< Sub-module type definitions */

/* my byte routines */
#include "byte_conversion.h"

/* hardware constants */
#include "esp32_defs.h"

/* === Keep These Local includes === */
#include "byte_conversion.h" /* Brings in unpackBytesToUint32()*/
#include "can_dispatch.h"    /* For sendRouteList*/
#include "can_platform.h"    /* Brings in can_msg_t */
#include "canbus_project.h"  /* my various CAN functions and structs */
#include "consumer_lookup.h" /**< Consumer lookup table */
#include "storage.h"         /**< NVS storage routines */
#include "task_consumer.h"
#include "task_output.h" /* Brings in getOutputTracker */
#include "task_twai.h"

#include "esp_log.h"

static const char *TAG = "task_consumer";

/*
 * =========================================================================
 * LOCAL VARIABLES
 * =========================================================================*/

/*
 * =========================================================================
 * LOCAL FUNCTION DECLARATIONS
 * =========================================================================*/
static uint32_t my_timestamp_ms(void);
static void routerLogAdapter(const char *msg);

static void setSwMomDur(can_msg_t *msg);

static void txSwitchState(const uint8_t *txUnitID, uint16_t txSwitchID,
                          const uint8_t swState);

static void buildSyntheticMessage(const router_action_t &action,
                                  can_msg_t &outMsg);

// static void prettyPrintMsg(const can_msg_t *msg);
static void handleCanRX(can_msg_t &message);

/*
 * =========================================================================
 * LOCAL FUNCTIONS
 * =========================================================================*/
static uint32_t my_timestamp_ms(void) {
  return millis(); // or esp_timer_get_time()/1000
}

static void routerLogAdapter(const char *msg) { ESP_LOGI("ROUTER", "%s", msg); }

static void setSwMomDur(can_msg_t *msg) {
  const uint8_t switchID = msg->data[4]; /* switch ID */
  const uint8_t momDur = msg->data[5];   /* momentary duration */
  if (SUBMODULE_INDEX_INVALID(switchID))
    return; /* invalid switch ID */

  subModule_t *sub =
      nodeGetActiveSubModule(switchID);   /* get submodule reference */
  sub->config.gpioOutput.param1 = momDur; /* update momentary duration */

  ESP_LOGI(TAG, "Momentary Duration: %d Switch: %d",
           sub->config.gpioOutput.param1, switchID);
}

static void txSwitchState(const uint8_t *txUnitID, uint16_t txSwitchID,
                          const uint8_t swState) {
  uint8_t dataBytes[8];
  static const uint8_t txDLC = 5;
  packUint32ToBytes(nodeGetInfo()->nodeID,
                    dataBytes); /* pack node ID into buffer */
  dataBytes[4] = (txSwitchID);  /* set switch ID */
  // dataBytes[5] = (swState); /* set switch state  */

  switch (swState) {

    case OUT_STATE_OFF: // switch off
      canEnqueueMessage(SW_SET_OFF_ID, dataBytes, SW_SET_OFF_DLC);
      break;
    case OUT_STATE_ON: // switch on
      canEnqueueMessage(SW_SET_ON_ID, dataBytes, SW_SET_ON_DLC);
      break;
    case OUT_STATE_MOMENTARY: // momentary press
      canEnqueueMessage(SW_MOM_PRESS_ID, dataBytes, SW_MOM_PRESS_DLC);
      break;
    default: // unsupported state
      ESP_LOGW(TAG, "Invalid switch state for transmission");
      break;
  }
}

static void buildSyntheticMessage(const router_action_t &action,
                                  can_msg_t &outMsg) {
  // Start with a clean message
  memset(&outMsg, 0, sizeof(outMsg));

  /* === MESSAGE SETUP === */
  /* copy CAN message ID */
  outMsg.identifier = action.actionMsgId;

  /* === EXTENDED PAYLOAD SUPPORT === */
  /* Extended 64-bit payload: overwrite entire payload */
  if (action.payload_flag == PAYLOAD_FLAG_64BIT) {
    uint64_t p = action.ext_payload64;

    outMsg.data_length_code = CAN_MAX_DLC; /* 8 bytes */

    packUint64ToBytes(p, outMsg.data);


    outMsg.isSynthetic = true;
    ESP_LOGV(TAG, "Synthetic 64-bit payload: %016llX", p);

    return;
  }

  /* Extended 32-bit payload: replace bytes 4–7 */
  if (action.payload_flag == PAYLOAD_FLAG_32BIT) {
    /* bytes 0–3 = nodeID */
    packUint32ToBytes(nodeGetInfo()->nodeID, &outMsg.data[0]);

    uint32_t p = action.ext_payload32;

    outMsg.data_length_code = CAN_MAX_DLC; /* 8 bytes */

    /* overwrite bytes 4–7 */
    outMsg.data[4] = (p >> 24) & 0xFF;
    outMsg.data[5] = (p >> 16) & 0xFF;
    outMsg.data[6] = (p >> 8) & 0xFF;
    outMsg.data[7] = (p >> 0) & 0xFF;

    outMsg.isSynthetic = true;
    ESP_LOGD(TAG, "Synthetic 32-bit payload from router: %u", p);

    return;
  }

  /* === LEGACY DATA MODE === */

  /* copy CAN message data length code if provided, otherwise use CAN_MAX_DLC */
  if (action.actionMsgDlc > 0) {
    outMsg.data_length_code = action.actionMsgDlc;
  } else {
    outMsg.data_length_code = CAN_MAX_DLC;
  }

  // Bytes 0–3 = NodeID (same as incoming messages)
  packUint32ToBytes(nodeGetInfo()->nodeID, &outMsg.data[0]);

  // Byte 4 = submodule index
  outMsg.data[4] = action.sub_idx;

  // Byte 5..7 = parameters from router
  outMsg.data[5] = action.param0;
  outMsg.data[6] = action.param1;
  outMsg.data[7] = action.param2;

  outMsg.isSynthetic = true; /* mark as synthetic */

  // ESP_LOGV(TAG,
  //          "Building synthetic message 0x%03X data 0x%02X 0x%02X 0x%02X 0x%02X",
  //          outMsg.identifier, outMsg.data[0], outMsg.data[1], outMsg.data[2],
  //          outMsg.data[3]);
}

static void handleCanRX(can_msg_t &message) {
  /* --- ROUTER ALWAYS SEES THE MESSAGE --- */
  router_action_t action = {0};
  can_msg_t msgToConsume = {0};

  /*
   * Use heartbeat messages to keep track of neighbors.
   * The message id range 0x780 through 0x7FF is reserved for
   * remote node identification.
   */
  if ((message.identifier >= 0x780 && message.identifier <= 0x7FF) &&
      message.data_length_code >= 4) {
    const uint32_t sender_node_id = packBytes32(&message.data[0]);
    nodeAddNeighbor(sender_node_id, get_ts());
  }

  /* pretty print the CAN message frame*/
  // if (message.identifier == 0x40C) {
  // ESP_LOGD(TAG, "[CAN RX] ID=0x%03X DLC=%d", message.identifier,
  //          message.data_length_code);
  // for (int i = 0; i < message.data_length_code; i++) {
  //   ESP_LOGD(TAG, "[CAN RX] Data=0x%02X ", message.data[i]);
  // }

  // }

  /* Pass all traffic through the router */
  bool takeAction = checkRoutes(&message, nodeGetNodeID(), &action);

  if (takeAction) {
    /* Synthetic messages are always for this node */
    buildSyntheticMessage(action, msgToConsume);
    consumeMsg(&msgToConsume);
    return; /* done */
  }

  /* --- NO ROUTER ACTION: APPLY NODE-ID FILTER HERE --- */

  bool msgForUs = false;

  if (message.data_length_code == 0) {
    /* broadcast frame */
    msgForUs = true;
  } else {
    uint32_t targetNodeID = unpackBytestoUint32(&message.data[0]);
    // Skip all 0s for now
    // if (targetNodeID == 0x00000000)
    //     msgForUs = true;  /* broadcast */
    // else
    if (targetNodeID == nodeGetNodeID())
      msgForUs = true; /* addressed to us */
  }

  if (!msgForUs)
    return; /* drop message */

  /* Pass original message to consumer */
  msgToConsume = message;
  consumeMsg(&msgToConsume);
}

/* =========================================================================
 *  Consumer Task
 * ========================================================================= */

static void TaskConsumer(void *pvParameters) {
  can_msg_t msg = {0};

  ESP_LOGI(TAG, "[RTOS] Consumer task starting...");

  ESP_LOGI(TAG, "[ROUTER] Loading routes from NVS...");

  loadRoutesFromNVS();

  const uint8_t routeCount = router_get_active_route_count();
  if (routeCount > 0) {
    ESP_LOGI(TAG, "[ROUTER] Load complete, %u active routes.", routeCount);
  } else {
    ESP_LOGW(TAG, "[ROUTER] No routes loaded.");
  }

  /* Block until TWAI driver is installed */
  while (!twaiIsDriverInstalled()) {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  ESP_LOGI(TAG, "[RTOS] Consumer task started.");

  node_state_t *ns = nodeGetState();

  for (;;) {
    /* Block until a CAN message arrives */
    if (xQueueReceive(canMsgRxQueue, &msg, portMAX_DELAY)) {
      /* update last RX timestamp */
      ns->last_rx_ts = millis();

      /* update total RX count */
      ns->total_rx_count++;

      /* reset watchdog stage */
      ns->bus_watchdog_stage = 0;

      handleCanRX(msg); // or consumeMsg(msg)
    }
  }
}

/* =========================================================================
 *  Task Startup
 * ========================================================================= */

void startTaskConsumer(void) {
  /* bind timestamp callback for can_router */
  router_set_timestamp_callback(my_timestamp_ms);

  /* bind log callback for can_router */
  router_set_log_callback(routerLogAdapter);

  /* Start the consumer task */
  xTaskCreate(TaskConsumer, "Task Consumer", TASK_CONSUMER_STACK_SIZE, NULL,
              tskNormalPriority, &xConsumerHandle);
}
