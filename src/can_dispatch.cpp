#include <Arduino.h>

#include "can_dispatch.h"
#include "task_twai.h"         /**< my TWAI task */
#include "can_router.h"        /**< CAN routing (subcriber) routines and constants */
#include "node_state.h"        /**< Node and sub-module state table */
#include "byte_conversion.h"   /**< Byte conversion functions */
#include "crc16.h"             /**< CRC16 functions */
#include "storage.h"           /**< NVS storage routines */

/* ========================================================================= 
  Private Variables
  ========================================================================= */

// static uint8_t introMsgPtr = 0;

/* can state machine flags */
static uint8_t FLAG_SEND_INTRODUCTION = 0;
static uint8_t FLAG_BEGIN_NORMAL_OPER = 0;
static uint8_t FLAG_HALT_NORMAL_OPER  = 0;
static uint8_t FLAG_SEND_HEALTHCHECK  = 0;
static uint8_t FLAG_SEND_NODECHECK    = 0;
static uint8_t FLAG_PRINT_TIMESTAMP   = 0;

/* network submodule variables and flags */
// static bool     pendingNetNodeValid   = false;
// static bool     pendingNetDataValid   = false;
// static uint32_t pendingRemoteNodeId   = 0;
// static uint8_t  pendingRemoteIndex    = 0;
// static uint8_t  pendingPersonalityId  = 0;
unsigned long previousMillis = 0;  /* will store last time a message was sent */

/* ========================================================================= 
  Public Variables
  ========================================================================= */
  

/* ========================================================================= 
  Local Functions                                       
  ========================================================================= */





/* ========================================================================= */
/* Public Functions                                       */
/* ========================================================================= */

/* send the router table to the CAN bus */
void sendRouteList()
{
  uint8_t       msgData[CAN_MAX_DLC] = {0};           /* wipe the buffer before using it */
  uint16_t      txMsgID              = 0;             /* Init to 0 */
  uint32_t      txMsgDLC             = CAN_MAX_DLC;   /* Init to 8 bytes */
  const uint8_t msgDataOffset        = 5;             /* skip the four bytes for node id and one byte for chunk id */
  uint8_t       routeCount           = 0;             /* Init to 0 */
  uint16_t      crc                  = 0xFFFF;        /* Initial value */

  // Count active routes
  for (int i = 0; i < MAX_ROUTES; i++) {
    if (g_routesCrc[i].in_use)
        routeCount++;
  }

  // Compute total chunks
  const uint8_t chunksPerRoute = ROUTE_CHUNKS_PER_ROUTE;
  const uint8_t totalChunks = routeCount * chunksPerRoute;

  /* compute route table crc16 */
  crc = crc16_ccitt((const uint8_t*)g_routes, sizeof(g_routes));

  /* Consistent 32-bit Node ID across all route frames */
  packUint32ToBytes(nodeGetInfo()->nodeID, &msgData[MSG_DATA_0]);

  /* Step 1: Send route list header */
  txMsgID  = ROUTE_LIST_BEGIN_ID;
  txMsgDLC = ROUTE_LIST_BEGIN_DLC;

  msgData[MSG_DATA_4] = routeCount;  /* set route count */
  msgData[MSG_DATA_5] = totalChunks; /* set total chunks */

  /* Send the message */
  canEnqueueMessage(txMsgID, msgData, txMsgDLC);

  /* Step 2: Send route list data */
  txMsgID  = ROUTE_LIST_DATA_ID;
  txMsgDLC = ROUTE_LIST_DATA_DLC;

  /* Reset the buffer, reload the node id */
  memset(msgData, 0, CAN_MAX_DLC);
  packUint32ToBytes(nodeGetInfo()->nodeID, &msgData[MSG_DATA_0]);

  uint8_t chunkIdx = 0;
  for (int idx = 0; idx < MAX_ROUTES; idx++) {
    if (g_routesCrc[idx].in_use) {

      /* Update CRC value with the CRC for this route entry */
      crc = crc16_ccitt_update(crc,
                                 (const uint8_t*)&g_routes[idx],
                                 sizeof(route_entry_t));

      /* Copy the route entry to the message buffer, 3 bytes at a time */
      const uint8_t *raw = (const uint8_t*)&g_routes[idx]; /* pointer to route data*/

      for (uint8_t chunk = 0; chunk < chunksPerRoute; chunk++) {
        msgData[MSG_DATA_4] = chunkIdx;         /* chunk index */
        msgData[MSG_DATA_5] = raw[chunk*3 + 0]; /* payload byte 0 */
        msgData[MSG_DATA_6] = raw[chunk*3 + 1]; /* payload byte 1 */
        msgData[MSG_DATA_7] = raw[chunk*3 + 2]; /* payload byte 2 */

        /* Send the message */
        canEnqueueMessage(txMsgID, msgData, txMsgDLC);

        chunkIdx++; /* increment chunk index */
      }
    }
  }

  /* Step 3: send end of route list message, include total routes, total chunks and crc16 */
  txMsgID  = ROUTE_LIST_END_ID;
  txMsgDLC = ROUTE_LIST_END_DLC;

  /* Reset the buffer, reload the node id */
  memset(msgData, 0, CAN_MAX_DLC);
  packUint32ToBytes(nodeGetInfo()->nodeID, &msgData[MSG_DATA_0]);

  msgData[MSG_DATA_4] = routeCount;           /* total route count      */
  msgData[MSG_DATA_5] = chunkIdx;             /* counter of chunks sent */
  msgData[MSG_DATA_6] = ((crc >> 8) & 0xFF);  /* set crc16 high byte    */
  msgData[MSG_DATA_7] = (crc & 0xFF);         /* set crc16 low byte     */

  /* Send the message */
  canEnqueueMessage(txMsgID, msgData, txMsgDLC);

} /* end sendRouteList() */

/**
 * @brief Send an introduction message about the node
 *
 * This function is called in response to a request from the master node
 * to introduce the node. The function will send an introduction message
 * with the node type and sub-module count. The function will
 * also send an introduction message for each sub-module with the
 * sub-module type and configuration.
 *
 * The function uses a cooldown to prevent spamming the bus every 100ms tick
 */
void sendIntroduction(int msgPtr) 
{
  uint8_t  msgData[CAN_MAX_DLC];

  uint32_t currentTick       = millis();
  uint32_t txMsgDLC          = CAN_MAX_DLC; /* Default to 8 bytes */
  uint16_t txMsgID           = nodeGetNodeType();

  bool     FLAG_VALID_CONFIG = nodeGetValidConfig();
  const nodeInfo_t* node     = nodeGetInfo();

  memset(&msgData, 0, sizeof(msgData)); /* wipe the buffer before using it */

  /* Consistent 32-bit Node ID across all intro frames */
  packUint32ToBytes(nodeGetNodeID(), &msgData[0]);

  // Serial.printf("TX INTRO: NODE 0x%08X SUBMODCNT %02u MSGPTR %d\n", node->nodeID, node->subModCnt, msgPtr);

/* 0: Node Identity */
  if (msgPtr == 0) {
    // uint16_t txCrc = FLAG_VALID_CONFIG ? crc16_ccitt((const uint8_t*)&node, sizeof(node)) : CRC_INVALID_CONFIG;
    
    /*  
     *  Check FLAG_VALID_CONFIG flag, if it is set send the CRC from a sanitized version 
     *  of the current nodeInfo_t struct. If the flag is not set send 0xFFFF 
     */
    const nodeInfo_t sanitized = makeSanitizedNodeInfo(nodeGetInfo());
    const uint16_t txCrc       = FLAG_VALID_CONFIG
          ? crc16_ccitt((const uint8_t*)&sanitized, sizeof(sanitized))
          : CRC_INVALID_CONFIG;


    txMsgID                    = nodeGetNodeType();
    msgData[4]                 = nodeGetSubModuleCount();
    msgData[5]                 = (uint8_t)(txCrc >> 8);
    msgData[6]                 = (uint8_t)(txCrc);

    // Serial.printf("TX INTRO: NODE 0x%08X SUBMODCNT %02u (Type: 0x%03X, CRC: 0x%04X)\n", node->nodeID, msgData[4], txMsgID, txCrc);
  }
/* >0: Sub-module Identity (Part A and Part B) */
  else {
    uint8_t modIdx = (uint8_t)((msgPtr - 1) / 2); /**< Map ptr to sub-module index */
    bool isPartB   = ((msgPtr - 1) % 2) != 0;      /**< Alternate A/B sequence */
    
    if (modIdx >= node->subModCnt) return;
    const subModule_t& sub    = *nodeGetSubModule(modIdx);
    const personalityDef_t* p = &runtimePersonalityTable[sub.personalityIndex]; /**< Pointer to the personality definition for this sub-module */

    const uint16_t dataMsgId  = p->dataMsgId;
    const uint8_t  dataMsgDlc = p->dataMsgDlc; 

    txMsgID                   = sub.introMsgId;

    if (!isPartB) {
        /* Part A: Configuration Data */
        msgData[4] = modIdx; /**< bits 0-6: index, bit 7: 0 (Part A) */
        msgData[5] = sub.config.rawConfig[0];
        msgData[6] = sub.config.rawConfig[1];
        msgData[7] = sub.config.rawConfig[2];
    } else {
        /* Part B: Telemetry/Operational Data */
        msgData[4] = modIdx | SUBMOD_PART_B_FLAG; /**< Set bit 7 to indicate Part B */
        msgData[5] = (uint8_t)(dataMsgId >> 8);
        msgData[6] = (uint8_t)(dataMsgId);
        /* Pack DLC (4 bits) and SaveState (1 bit) into byte 7 */
        msgData[7] = (dataMsgDlc & 0x0F) | (sub.submod_flags ? SUBMOD_PART_B_FLAG : 0x00);
    }

    if (txMsgID == 0) {
        return;  /* error condition, msg ID is invalid, exit the routine */
    }

    Serial.printf("TX INTRO: MOD 0x%03X at Idx %i (Cfg: %02X %02X %02X)\n",
                  txMsgID, msgData[4], msgData[5], msgData[6], msgData[7]);
  }
  /* put the message on the bus */
  canEnqueueMessage(txMsgID, msgData, txMsgDLC);
//   prepareMessage(txMsgID, msgData, txMsgDLC);

} /* end sendIntroduction */


void canDispatchStartIntroduction()
{
    /** Start the introduction process */ 
    sendIntroduction(0);  /* start at index 0*/
};

void canDispatchProcessTimers()
{

};

bool canDispatchHasPendingNetworkData()
{
  return false;
};