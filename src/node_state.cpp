#include "node_state.h"
#include "esp_log.h"

static const char *TAG = "node_state";

/* ============================================================================
 *  CONSTANTS
 * ============================================================================
 */

/* ============================================================================
 *  PRIVATE VARIABLES
 * ============================================================================
 */

/** Flag indicating that the node config is valid */
bool FLAG_NODE_CONFIG_VALID = false;
uint16_t nodeCRC = 0xFFFF; /* start with an invalid CRC */

/* ============================================================================
 *  GLOBAL VARIABLES
 * ============================================================================
 */

/** Runtime data storage */
// submoduleRuntime_t g_subRuntime[MAX_SUB_MODULES];

/** Current node configuration */
nodeInfo_t node;

/** Local node state tracking */
node_state_t node_state;

/** Bus neighbor list */
node_neighbor_t node_neighbors[MAX_NEIGHBORS];

/** Runtime data storage */
runTime_t runtime_state[MAX_SUB_MODULES];

/* ============================================================================
 *  PRIVATE FUNCTIONS
 * ============================================================================
 */

/* ============================================================================
 *  PUBLIC API FUNCTIONS
 * ============================================================================
 */

/* Print memory information about node structures to the Serial console */
void nodePrintStructInfo(void) {
  /* Debug check for memory alignment */
  ESP_LOGI(
      TAG,
      "[INIT] Struct Sizes - nodeInfo_t: %d, subModule_t: %d, runTime_t: %d",
      sizeof(nodeInfo_t), sizeof(subModule_t), sizeof(runTime_t));
}

/** Initialize the node state array */
void nodeInit(void) {
  /* Clear entire node state */
  memset(&node, 0, sizeof(nodeInfo_t));

  /* Mark all submodules as unused */
  memset(node.subModule, 0xFF, sizeof(node.subModule));

  /* Mark all runtimes as unused */
  memset(runtime_state, 0, sizeof(runtime_state));

  /* Clear node state */
  node_state = {0};

  /* Initialize node state */
  node_state.active = true;
  node_state.node_id = node.nodeID;
  node_state.last_epoch = 0;

  /* Initialize neighbor list */
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    node_neighbors[i].active = false;
    node_neighbors[i].node_id = 0;
    node_neighbors[i].last_seen = 0;
  }
}

/** Read the MAC address from hardware, store it in nodeInfo_t */
void nodeReadMacAddress(void) {
  uint8_t baseMac[6] = {};
  esp_efuse_mac_get_default(baseMac); // factory-burned MAC

  // Extract bytes 2–5 as a big-endian uint32_t
  node.nodeID = (static_cast<uint32_t>(baseMac[2]) << 24) |
                (static_cast<uint32_t>(baseMac[3]) << 16) |
                (static_cast<uint32_t>(baseMac[4]) << 8) |
                (static_cast<uint32_t>(baseMac[5]));

  ESP_LOGI(TAG, "[INIT] Node ID extracted: 0x%02X%02X%02X%02X  (0x%08X)",
           baseMac[2], baseMac[3], baseMac[4], baseMac[5], node.nodeID);
}

bool nodeStateIsInitialized(void) { return (node.nodeID != 0); }
/** @brief Return a pointer to the nodeInfo_t struct */
nodeInfo_t *nodeGetInfo() { return &node; }

/** @brief Return the node ID as a 32-bit unsigned integer */
uint32_t nodeGetNodeID() { return node.nodeID; }

/** @brief Return the node type as a 16-bit unsigned integer */
uint16_t nodeGetNodeType() { return node.nodeTypeMsg; }

/* CRC accessor functions */
uint16_t nodeGetCRC() { return nodeCRC; }
void nodeSetCRC(uint16_t crc) { nodeCRC = crc; }

/* Config flag accessor functions */
bool nodeSetValidConfig() { return FLAG_NODE_CONFIG_VALID; }
void nodeSetValidConfig(bool valid) { FLAG_NODE_CONFIG_VALID = valid; }

/**
 * @brief Return a pointer to the sub-module's runtime data at the given index.
 *
 * @param sub_idx The index of the sub-module to retrieve the runtime data from.
 * @return A pointer to the runtime data of the sub-module at the given index.
 *         If the sub-module does not exist, returns NULL.
 */
runTime_t *nodeGetRuntime(const uint8_t sub_idx) {
  if (sub_idx >= MAX_SUB_MODULES)
    return NULL;
  return &runtime_state[sub_idx];
}

/**
 * @brief Return a pointer to the sub-module at the given index.
 *
 * @param sub_idx The index of the sub-module to check.
 * @return A pointer to the sub-module, returns NULL if the
 * index is out of bounds.
 */
subModule_t *nodeGetSubModule(const uint8_t sub_idx) {
  if (sub_idx >= MAX_SUB_MODULES)
    return NULL;

  return &node.subModule[sub_idx];
}

/**
 * @brief Return a pointer to the sub-module at the given index
 * if it is active, NULL otherwise.
 *
 * @param sub_idx The index of the sub-module to check.
 * @return A pointer to the sub-module if it is active, NULL otherwise.
 */
subModule_t *nodeGetActiveSubModule(const uint8_t sub_idx) {
  subModule_t *sub = nodeGetSubModule(sub_idx);
  /* Submodule not found, exit function */
  if (!sub)
    return NULL;

  /* Submodule not active, exit function */
  if (sub->personalityIndex == 0xFF)
    return NULL;

  return sub;
}

/** Producer flags reader */
uint8_t *nodeGetProducerFlags(const uint8_t sub_idx) {
  return &node.subModule[sub_idx].producer_flags;
}

/** Producer flags writer */
void nodeSetProducerFlags(const uint8_t sub_idx, uint8_t flags) {
  node.subModule[sub_idx].producer_flags = flags;
}

/** Submodule flags reader */
uint8_t *nodeGetSubmodFlags(const uint8_t sub_idx) {
  return &node.subModule[sub_idx].submod_flags;
}

/** Submodule flags writer */
void nodeSetSubmodFlags(const uint8_t sub_idx, uint8_t flags) {
  node.subModule[sub_idx].submod_flags = flags;
}

/** Router flags reader */
uint8_t *nodeGetRouterFlags(const uint8_t sub_idx) {
  return &node.subModule[sub_idx].router_flags;
}

/** Router flags writer */
void nodeSetRouterFlags(const uint8_t sub_idx, uint8_t flags) {
  node.subModule[sub_idx].router_flags = flags;
}

/** @brief Count, update and return the number of active submodules */
uint8_t nodeGetActiveSubModuleCount(void) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_SUB_MODULES; i++) {
    subModule_t *sub = nodeGetActiveSubModule(i);
    if (!sub)
      continue;

    count++;
  }

  /* Keep runtime count in sync */
  node.subModCnt = count;

  return count;
}

/** @brief Count and return the number of active personalities */
uint8_t nodeGetActivePersonalityCount(void) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_RUNTIME_PERSONALITIES; i++) {
    if (runtimePersonalityTable[i].personalityId != 0xFF)
      count++;
  }

  return count;
}

void nodeIncSubModuleCount(void) {
  /* this function used to manipulate the runtime count
  directly, which is unsafe. Instead, call the recount
  function */
  uint8_t count = nodeGetActiveSubModuleCount();
}

/** * @brief Return true if the submodule index is valid, does not care
 * about empty slots only valid indexes
 */
bool nodeIsValidSubmodule(uint8_t index) {
  return !(SUBMODULE_INDEX_INVALID(index));
}

/** * @brief Return true if the submodule index is valid and not empty */
bool nodeIsActiveSubmodule(uint8_t index) {
  return ((index < MAX_SUB_MODULES) &&
          (nodeGetSubModule(index)->personalityIndex != 0xFF));
}

/** * @brief Return true if the personality index is in-bounds */
bool nodeIsValidPersonality(uint8_t personalityIndex) {
  return (personalityIndex < MAX_RUNTIME_PERSONALITIES);
}

/** * @brief Return true if the personality index is active */
bool nodeIsActivePersonality(uint8_t personalityIndex) {
  return ((personalityIndex < MAX_RUNTIME_PERSONALITIES) &&
          (nodeGetPersonality(personalityIndex)->personalityId != 0xFF));
}

/**
 * @brief Return a pointer to the personality at the given index.
 *
 * @param personalityIndex The index of the personality to retrieve.
 * @return A pointer to the personality at the given index, or NULL
 *  if the index is out of bounds.
 */
const personalityDef_t *nodeGetPersonality(uint8_t personalityIndex) {
  if (PERSONALITY_INDEX_INVALID(personalityIndex))
    return NULL;

  return &runtimePersonalityTable[personalityIndex];
}

/**
 * @brief Return a pointer to the personality at the given index.
 * @param personalityIndex uint8_t index of the personality to retrieve.
 * @return A pointer to the personalityDef_t structure at the given index
 * if the index is valid and the personality is active, NULL otherwise.
 */
const personalityDef_t *nodeGetActivePersonality(uint8_t personalityIndex) {
  if (!nodeIsActivePersonality(personalityIndex))
    return NULL;

  return &runtimePersonalityTable[personalityIndex];
}

/** Pretty print nodeInfo_t */
/**
 * @brief Prints a pretty formatted version of the nodeInfo_t structure to the
 * Serial console.
 *
 * @param node Pointer to the nodeInfo_t structure to print.
 */
void printNodeInfo(const nodeInfo_t *node) {
  ESP_LOGD("NODEINFO", "--------- nodeInfo_t ---------");
  ESP_LOGD("NODEINFO", "NodeInfo:");
  ESP_LOGD("NODEINFO", "  nodeID: 0x%08X", node->nodeID);
  ESP_LOGD("NODEINFO", "  nodeTypeMsg: 0x%03X", node->nodeTypeMsg);
  ESP_LOGD("NODEINFO", "  nodeTypeDLC: %u", node->nodeTypeDLC);
  ESP_LOGD("NODEINFO", "  subModCnt: %u", node->subModCnt);

  ESP_LOGD("NODEINFO", "  subModule array:");
  for (uint8_t i = 0; i < node->subModCnt; i++) {

    const personalityDef_t *p = nodeGetPersonality(
        node->subModule[i]
            .personalityIndex); // &runtimePersonalityTable[node->subModule[i].personalityIndex]

    ESP_LOGD("NODEINFO", "     subModule[%u]:", i);
    ESP_LOGD("NODEINFO", "       personalityId: %u",
             node->subModule[i].personalityId);
    ESP_LOGD("NODEINFO", "       personalityIndex: %u",
             node->subModule[i].personalityIndex);

    ESP_LOGD("NODEINFO", "       (p->) dataMsgId: 0x%03X", p->dataMsgId);
    ESP_LOGD("NODEINFO", "       (p->) dataMsgDlc: %u", p->dataMsgDlc);
    ESP_LOGD("NODEINFO", "       (p->) gpio Pin: %u", p->gpioPin);
    ESP_LOGD("NODEINFO", "       (p->) flags: 0x%02X", p->flags);

    if (node->subModule[i].personalityId == PERS_GPIO_INPUT) {
      ESP_LOGD("NODEINFO", "       gpio_input flags: 0x%02X",
               node->subModule[i].config.gpioInput.flags);
      ESP_LOGD("NODEINFO", "       gpio_input debounce_ms: %u",
               node->subModule[i].config.gpioInput.debounce_ms);
      ESP_LOGD("NODEINFO", "       gpio_input reserved: 0x%02X",
               node->subModule[i].config.gpioInput.reserved);
    } else {
      ESP_LOGD("NODEINFO", "       config bytes 0x%02X 0x%02X 0x%02X",
               node->subModule[i].config.rawConfig[0],
               node->subModule[i].config.rawConfig[1],
               node->subModule[i].config.rawConfig[2]);
    }

    ESP_LOGD("NODEINFO", "       introMsgId: 0x%03X",
             node->subModule[i].introMsgId);
    ESP_LOGD("NODEINFO", "       introMsgDLC: %u",
             node->subModule[i].introMsgDLC);
    ESP_LOGD("NODEINFO", "       submod_flags: 0x%02X",
             node->subModule[i].submod_flags);
    ESP_LOGD("NODEINFO", "       router_flags: 0x%02X",
             node->subModule[i].router_flags);

    ESP_LOGD("NODEINFO", "       producer_flags: 0x%02X",
             node->subModule[i].producer_flags);
    ESP_LOGD("NODEINFO", "       producer_kind: %u",
             node->subModule[i].producer_kind);
    ESP_LOGD("NODEINFO", "       producer_period_ms: %u",
             node->subModule[i].producer_period_ms);

    ESP_LOGD("NODEINFO", "       runTime:");
    ESP_LOGD("NODEINFO", "         last_change_ms: %u",
             runtime_state[i].last_change_ms);
    ESP_LOGD("NODEINFO", "         valueU32: %u", runtime_state[i].valueU32);
    ESP_LOGD("NODEINFO", "         last_published_value: %u",
             runtime_state[i].last_published_value);
  }

  ESP_LOGD("NODEINFO", "--------- nodeInfo_t ---------");
}

/**
 * @brief Get the current epoch time from the system clock.
 *
 * This function reads the current time from the ESP32 system clock
 * and returns it as a uint32_t representing the number of seconds
 * since the epoch (January 1, 1970, 00:00:00 UTC).
 *
 * @return uint32_t The current epoch time in seconds.
 */

// uint32_t getEpochTime() {
//   /* Get time from the system clock and return it as a uint32_t */
//   struct timespec newTime;

//   clock_gettime(CLOCK_REALTIME, &newTime); /* Read time from ESP32 clock*/

//   return (uint32_t)newTime.tv_sec;
// }

/* -----------------------------------------------------------
 ** @details nodeGetInternalPeriodicProducers()
 *
 * Returns a curated list of submodule indices that:
 *   - exist
 *   - are INTERNAL
 *   - are ACTIVE
 *   - have PUBLISH_ENABLED
 *   - are PERIODIC producers
 *
 * No timing logic here — node_state does not own clocks.
 * ----------------------------------------------------------- */
uint8_t nodeGetInternalPeriodicProducers(uint8_t *outList, uint8_t max) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_SUB_MODULES; i++) {
    if (count >= max)
      break;

    subModule_t *sub = nodeGetActiveSubModule(i);

    if (!sub)
      continue;

    if (!(sub->submod_flags & SUBMOD_FLAG_INTERNAL))
      continue;

    if (!(sub->producer_flags & PRODUCER_FLAG_ACTIVE))
      continue;

    if (!(sub->producer_flags & PRODUCER_FLAG_PUBLISH_ENABLED))
      continue;

    /* If producer period is too short, or producer is a one-shot, skip it */
    if ((sub->producer_period_ms == PRODUCER_PERIOD_ONCE) ||
        (sub->producer_period_ms < PRODUCER_PERIOD_1000MS))
      continue;

    outList[count++] = i;
  }

  return count;
}

/**
 * @brief Returns a pointer to the node state object.
 *
 * @return A pointer to the node_state_t object.
 *
 * The node state object contains basic information about the current
 * state of the node. It is primarily used for the TWAI watchdog.
 *
 */
node_state_t *nodeGetState() { return &node_state; }

void nodeIncRxDropCount(void) { node_state.total_rx_drops++; }
void nodeIncTxDropCount(void) { node_state.total_tx_drops++; }

/**
 * @brief Returns a pointer to the node's neighbor table.
 *
 * The neighbor table contains entries for each known neighbor node,
 * including the node ID, last seen timestamp, and active flag.
 *
 * @return A pointer to the node's neighbor table.
 *
 */
node_neighbor_t *nodeGetNeighborTable(void) { return node_neighbors; }

/**
 * @brief Returns the number of neighbor nodes currently tracked by the node.
 *
 * This function returns the number of neighbor nodes that have been
 * discovered and are currently being tracked by the node. The number
 * of neighbor nodes is determined by checking the node_id value in the
 * neighbor table.
 *
 * @return The number of neighbor nodes currently tracked by the node.
 */
uint8_t nodeGetNeighborCount(void) {
  uint8_t count = 0;

  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (node_neighbors[i].node_id != 0)
      count++;
  }

  return count;
}

/**
 * @brief Returns the number of active neighbor nodes currently tracked by the
 * node.
 *
 * This function returns the number of neighbor nodes that have been
 * discovered and are currently being tracked by the node. The number
 * of neighbor nodes is determined by checking the node_id value and
 * the active flag in the neighbor table.
 *
 * @return The number of active neighbor nodes currently tracked by the node.
 */
uint8_t nodeGetActiveNeighborCount(void) {
  uint8_t count = 0;

  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if ((node_neighbors[i].node_id != 0) && (node_neighbors[i].active))
      count++;
  }

  return count;
}

/**
 * @brief Returns a pointer to the neighbor node with the given node_id.
 *
 * This function searches the neighbor table for a matching node_id and
 * returns a pointer to the matching node_neighbor_t entry. The active
 * flag is not considered in the search, so this function will return a
 * pointer regardless of whether the neighbor is currently active or not.
 *
 * @param node_id The node ID to search for.
 * @return A pointer to the matching node_neighbor_t entry if found, NULL
 * otherwise.
 */
node_neighbor_t *nodeGetNeighbor(uint32_t node_id) {
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    /* match on node_id alone, not active flag */
    if (node_neighbors[i].node_id == node_id) {
      return &node_neighbors[i];
    }
  }

  return NULL;
}

/**
 * @brief Adds a neighbor node to the node's neighbor table.
 *
 * This function rejects the local node and broadcast messages (node_id == 0).
 * It first checks if there is an existing entry in the neighbor table with
 * the matching node_id. If so, it updates the active flag and last seen
 * time. If not, it inserts the new node_id into the first inactive slot.
 * If the neighbor table is full, it logs a warning message.
 *
 * @param node_id The node ID to add to the neighbor table.
 * @param now The current timestamp in milliseconds.
 */
void nodeAddNeighbor(uint32_t node_id, uint32_t now) {
  /* reject the local node, and broadcast messages (node_id == 0)*/
  if ((node_id == node.nodeID) || (node_id == 0)) {
    ESP_LOGW(TAG, "Node ID is invalid: %lu", node_id);
    return;
  }

  /* First pass: update existing entry */
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    /* match on node_id alone, not active flag */
    if (node_neighbors[i].node_id == node_id) {
      /* Update active flag and last seen time */
      node_neighbors[i].active = true;
      node_neighbors[i].last_seen = now;
      return;
    }
  }

  /* Second pass: insert into first inactive slot */
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    /* an empty slot will have 0 for a node_id */
    if (node_neighbors[i].node_id == 0) {
      node_neighbors[i].active = true;
      node_neighbors[i].node_id = node_id;
      node_neighbors[i].last_seen = now;
      return;
    }
  }

  /* Optional: table full */
  ESP_LOGW(TAG, "Neighbor table full, cannot add node_id=%lu", node_id);
}

void nodePruneNeighbors(uint32_t now, uint32_t timeout_ms) {
  for (int i = 0; i < MAX_NEIGHBORS; i++) {

    if (node_neighbors[i].active == true) {
      uint32_t age = now - node_neighbors[i].last_seen;

      if (age > timeout_ms) {
        /* Stage 1: mark inactive */
        node_neighbors[i].active = false;
      }
    } else {
      /* Stage 2: wipe inactive entries older than 2× timeout */
      uint32_t age = now - node_neighbors[i].last_seen;

      if (age > (timeout_ms * 2)) {
        node_neighbors[i].node_id = 0;
        node_neighbors[i].last_seen = 0;
      }
    }
  }
}

void nodeSetRtcFlag(bool use_rtc) { node_state.use_rtc = use_rtc; }

bool nodeGetRtcFlag(void) { return node_state.use_rtc; }

uint32_t get_ts(void) 
{
  if (node_state.use_rtc) {
    /* read epoch time from rtc */
    return (uint32_t)time(NULL);
  } else {
    return millis();
  }
}
