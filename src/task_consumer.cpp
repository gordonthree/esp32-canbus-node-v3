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
  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */
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

  outMsg.identifier = action.actionMsgId;
  if (action.actionMsgDlc > 0)
  {
    outMsg.data_length_code = action.actionMsgDlc;
  }
  else
  {
    outMsg.data_length_code = CAN_MAX_DLC; // Always 8 for consumer actions
  }

  // Bytes 0–3 = NodeID (same as incoming messages)
  packUint32ToBytes(nodeGetInfo()->nodeID, &outMsg.data[0]);

  // Byte 4 = submodule index
  outMsg.data[4]     = action.sub_idx;
   
  // Byte 5..7 = parameters from router
  outMsg.data[5]     = action.param[0];
  outMsg.data[6]     = action.param[1];
  outMsg.data[7]     = action.param[2];

  outMsg.isSynthetic = true; /* mark as synthetic */

  // ESP_LOGD(TAG, "Building synthetic message 0x%03X data 0x%02X 0x%02X 0x%02X 0x%02X",
          //  outMsg.identifier, outMsg.data[0], outMsg.data[1], outMsg.data[2], outMsg.data[3]);
}

static void prettyPrintMsg(const can_msg_t *msg)
{
  /* Enough space for "0xFF " * 8 + null */
  char dataBuf[5 * CAN_MAX_DLC + 1]; /* 5 bytes of string per byte + null */
  int pos = 0;

  for (int i = 0; i < msg->data_length_code; i++)
  {
    pos += snprintf(&dataBuf[pos],
                    sizeof(dataBuf) - pos,
                    "0x%02X ",
                    msg->data[i]);
    if (pos >= sizeof(dataBuf))
      break;
  }

  ESP_LOGD("CANMSG",
           "ID: 0x%03X DLC: %d DATA: %s",
           msg->identifier,
           msg->data_length_code,
           dataBuf);
}

static void handleCanRX(can_msg_t &message)
{
  bool msgForUs = false;

  /* process broadcast ARGB color command */
  // if ((message.identifier == SET_ARGB_STRIP_COLOR_ID) || (message.identifier == SET_ARGB_BUTTON_COLOR_ID)) {
  //   handleColorCommand(message);
  // }

  if (message.data_length_code == 0)
  {
    /* general broadcast message is valid, message has no node id assigned */
    msgForUs = true; /* message is for us */
    ESP_LOGD(TAG, "[CONSUMER] RX BROADCAST MSG: 0x%x NO DATA", message.identifier);
  }
  else
  {
    /* set flag if message is for us */
    const uint32_t targetNodeID = unpackBytestoUint32(&message.data[0]);
    msgForUs = (targetNodeID == nodeGetInfo()->nodeID);
  }

  if (!msgForUs)
  {
    return; // message is not for us
  }
  
  /* debug: dump message data into a string buffer and then send to ESP_LOGD */
  ESP_LOGD(TAG, "[CONSUMER] RX MSG PRE-ROUTER");
  prettyPrintMsg(&message);

  /* prepare router library action buffer */
  router_action_t action = {0};

  /* zero-init consumer buffer */
  can_msg_t msgToConsume = {0};

  /* hand off message to the router library */
  bool takeAction = checkRoutes(&message, &action);

  /* decide if we generate a synthetic message or use the original */
  if (takeAction)
  {
    /* message router indicates we need to generate a synthetic message */
    ESP_LOGI(TAG, "[ROUTER] ACTION: 0x%03X", action.actionMsgId);
    buildSyntheticMessage(action, msgToConsume);
    ESP_LOGD(TAG, "[CONSUMER] RX MSG POST-ROUTER:Synthetic Message Returned");
    prettyPrintMsg(&msgToConsume);
  }
  else
  {
    /* pass along original message */
    msgToConsume = message;
    /* debug: dump message data */
    ESP_LOGD(TAG, "[CONSUMER] RX MSG POST-ROUTER:No Action");
    prettyPrintMsg(&msgToConsume);
  }

  consumeMsg(&msgToConsume); /* pass message to consumer */

} // end of void handleCanRX()

/* =========================================================================
 *  Consumer Task
 * ========================================================================= */

static void TaskConsumer(void *pvParameters)
{
  can_msg_t msg;

  ESP_LOGI(TAG, "[RTOS] Consumer task starting...");

  ESP_LOGI(TAG, "[ROUTER] Loading routes from NVS...");
  vTaskDelay(pdMS_TO_TICKS(10));
  loadRoutesFromNVS();
  vTaskDelay(pdMS_TO_TICKS(10));

  const uint8_t routeCount = routerGetRouteCount();
  if (routeCount > 0) {
    ESP_LOGI(TAG, "[ROUTER] Load complete, %d routes loaded.", routerGetRouteCount());
    prettyPrintRoutes();
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
