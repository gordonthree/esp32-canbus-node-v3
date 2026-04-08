/**
 * @file task_consumer.cpp
 * @brief CAN consumer logic
 *
 */

/* === Framework includes === */
#include <Arduino.h>

/* === Standard library includes === */
#include <stdio.h>
#include <stddef.h>

/* === Local includes === */
#include "can_router.h"        /**< CAN routing (subcriber) routines and constants */
#include "can_producer.h"      /**< CAN producer routines and constants */
#include "personality_table.h" /**< Node and sub-module personality table */
#include "node_state.h"        /**< Node and sub-module state table */
#include "submodule_types.h"   /**< Sub-module type definitions */

/* my byte routines */
#include "byte_conversion.h"

/* hardware constants */
#include "esp32_defs.h"

/* === Keep These Local includes === */
#include "can_dispatch.h"   /* For sendRouteList*/
#include "canbus_project.h" /* my various CAN functions and structs */
#include "task_consumer.h"
#include "can_platform.h" /* Brings in can_msg_t */
#include "task_twai.h"
#include "task_output.h"     /* Brings in getOutputTracker */
#include "byte_conversion.h" /* Brings in unpackBytesToUint32()*/
#include "storage.h"         /**< NVS storage routines */
#include "consumer_lookup.h" /**< Consumer lookup table */

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

static void setSwMomDur(can_msg_t *msg);

static void txSwitchState(const uint8_t *txUnitID,
                          uint16_t txSwitchID,
                          const uint8_t swState);

static void buildSyntheticMessage(const router_action_t &action,
                                  can_msg_t &outMsg);

static void prettyPrintMsg(const can_msg_t *msg);
static void handleCanRX(can_msg_t &message);

/*
 * =========================================================================
 * LOCAL FUNCTIONS
 * =========================================================================*/

static void setSwMomDur(can_msg_t *msg)
{
  const uint8_t switchID = msg->data[4]; /* switch ID */
  const uint8_t momDur = msg->data[5];   /* momentary duration */
  if (SUBMODULE_INDEX_INVALID(switchID))
    return; /* invalid switch ID */

  subModule_t *sub = nodeGetActiveSubModule(switchID); /* get submodule reference */
  sub->config.gpioOutput.param1 = momDur;        /* update momentary duration */

  ESP_LOGI(TAG, "Momentary Duration: %d Switch: %d",
           sub->config.gpioOutput.param1, switchID);
}

static void txSwitchState(const uint8_t *txUnitID, uint16_t txSwitchID, const uint8_t swState)
{
  uint8_t dataBytes[8];
  static const uint8_t txDLC = 5;
  packUint32ToBytes(nodeGetInfo()->nodeID, dataBytes); /* pack node ID into buffer */
  dataBytes[4] = (txSwitchID);                         /* set switch ID */
  // dataBytes[5] = (swState); /* set switch state  */

  switch (swState)
  {

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
                                  can_msg_t &outMsg)
{
  // Start with a clean message
  memset(&outMsg, 0, sizeof(outMsg));

  /* copy CAN message ID */
  outMsg.identifier = action.actionMsgId; 

  /* copy CAN message data length code if provided, otherwise use CAN_MAX_DLC */ 
  if (action.actionMsgDlc > 0)
  {
    outMsg.data_length_code = action.actionMsgDlc;
  }
  else
  {
    outMsg.data_length_code = CAN_MAX_DLC;
  }

  // Bytes 0–3 = NodeID (same as incoming messages)
  packUint32ToBytes(nodeGetInfo()->nodeID, &outMsg.data[0]);

  // Byte 4 = submodule index
  outMsg.data[4]     = action.sub_idx;
   
  // Byte 5..7 = parameters from router
  outMsg.data[5]     = action.param0;
  outMsg.data[6]     = action.param1;
  outMsg.data[7]     = action.param2;

  outMsg.isSynthetic = true; /* mark as synthetic */

  ESP_LOGD(TAG, "Building synthetic message 0x%03X data 0x%02X 0x%02X 0x%02X 0x%02X",
           outMsg.identifier, outMsg.data[0], outMsg.data[1], outMsg.data[2], outMsg.data[3]);
}

static void handleCanRX(can_msg_t &message)
{
    /* --- ROUTER ALWAYS SEES THE MESSAGE --- */
    router_action_t action = {0};
    can_msg_t msgToConsume = {0};

    bool takeAction = checkRoutes(&message, &action);

    if (takeAction)
    {
        /* Synthetic messages are always for this node */
        buildSyntheticMessage(action, msgToConsume);
        consumeMsg(&msgToConsume);
        return; /* done */
    }

    /* --- NO ROUTER ACTION: APPLY NODE-ID FILTER HERE --- */

    bool msgForUs = false;

    if (message.data_length_code == 0)
    {
        /* broadcast frame */
        msgForUs = true;
    }
    else
    {
        uint32_t targetNodeID = unpackBytestoUint32(&message.data[0]);
      // Skip all 0s for now
        // if (targetNodeID == 0x00000000)
        //     msgForUs = true;  /* broadcast */
        // else 
        if (targetNodeID == nodeGetNodeID())
            msgForUs = true;  /* addressed to us */
    }

    if (!msgForUs)
        return;  /* drop message */

    /* Pass original message to consumer */
    msgToConsume = message;
    consumeMsg(&msgToConsume);
}


/* =========================================================================
 *  Consumer Task
 * ========================================================================= */

static void TaskConsumer(void *pvParameters)
{
  can_msg_t msg;

  ESP_LOGI(TAG, "[RTOS] Consumer task starting...");

  ESP_LOGI(TAG, "[ROUTER] Loading routes from NVS...");
  loadRoutesFromNVS();

  const uint8_t routeCount = routerGetRouteCount();
  if (routeCount > 0) {
    ESP_LOGI(TAG, "[ROUTER] Load complete, %d routes loaded.", routerGetRouteCount());
    // prettyPrintRoutes();
  }
  else {
    ESP_LOGW(TAG, "[ROUTER] No routes loaded.");
  }

  
  /* Block until TWAI driver is installed */
  while (!twaiIsDriverInstalled())
  {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  ESP_LOGI(TAG, "[RTOS] Consumer task started.");

  for (;;)
  {
    /* Block until a CAN message arrives */
    if (xQueueReceive(canMsgRxQueue, &msg, portMAX_DELAY))
    {
      handleCanRX(msg); // or consumeMsg(msg)
    }
  }
}

/* =========================================================================
 *  Task Startup
 * ========================================================================= */

void startTaskConsumer(void)
{
  xTaskCreate(
      TaskConsumer,
      "Task Consumer",
      TASK_CONSUMER_STACK_SIZE,
      NULL,
      tskNormalPriority,
      &xConsumerHandle);
}
