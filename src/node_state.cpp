#include "node_state.h"

/* ============================================================================
 *  CONSTANTS
 * ============================================================================ */ 


/* ============================================================================
 *  PRIVATE VARIABLES
 * ============================================================================ */  

 /** Flag indicating that the node config is valid */
bool FLAG_NODE_CONFIG_VALID = false;
uint16_t nodeCRC = 0xFFFF; /* start with an invalid CRC */



/* ============================================================================
 *  GLOBAL VARIABLES
 * ============================================================================ */ 

 /** Runtime data storage */
// submoduleRuntime_t g_subRuntime[MAX_SUB_MODULES];

/** Current node configuration */
nodeInfo_t node; 

 /* ============================================================================
 *  PRIVATE FUNCTIONS
 * ============================================================================ */ 


 /* ============================================================================
 *  PUBLIC API FUNCTIONS
 * ============================================================================ */ 

/* Print memory information about node structures to the Serial console */
void nodePrintStructInfo(void) 
{
  /* Debug check for memory alignment */
  Serial.printf("\n[DEBUG] Struct Sizes - nodeInfo_t: %d, subModule_t: %d, runTime_t: %d\n",
              sizeof(nodeInfo_t), sizeof(subModule_t), sizeof(runTime_t));
}

/** Initialize the node state array */
void nodeInit(void) { memset(&node, 0, sizeof(nodeInfo_t)); }

/** Read the MAC address from hardware, store it in nodeInfo_t */
void nodeReadMacAddress(void)
{
    uint8_t baseMac[6] = {};
    esp_efuse_mac_get_default(baseMac);   // factory-burned MAC

    // Extract bytes 2–5 as a big-endian uint32_t
    node.nodeID =
        (static_cast<uint32_t>(baseMac[2]) << 24) |
        (static_cast<uint32_t>(baseMac[3]) << 16) |
        (static_cast<uint32_t>(baseMac[4]) << 8)  |
        (static_cast<uint32_t>(baseMac[5]));

    Serial.printf("[INIT] Node ID extracted: 0x%02X%02X%02X%02X  (0x%08X)\n",
                  baseMac[2], baseMac[3], baseMac[4], baseMac[5],
                  node.nodeID);
}


nodeInfo_t* nodeGetInfo() { return &node; }
uint32_t nodeGetNodeID() { return node.nodeID; }
uint16_t nodeGetNodeType() { return node.nodeTypeMsg; }

/* CRC accessor functions */
uint16_t nodeGetCRC() { return nodeCRC; }
void nodeSetCRC(uint16_t crc) { nodeCRC = crc; }

/* Config flag accessor functions */
bool nodeSetValidConfig() { return FLAG_NODE_CONFIG_VALID; }
void nodeSetValidConfig(bool valid) { FLAG_NODE_CONFIG_VALID = valid; }

/** Runtime array accessor */
runTime_t* nodeGetRuntime(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].runTime; 
}

/** Submodule array accessor */
subModule_t* nodeGetSubModule(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx];
}

/** Producer flags reader */
uint8_t* nodeGetProducerFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].producer_flags;  
}

/** Producer flags writer */
void nodeSetProducerFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].producer_flags = flags;
}

/** Submodule flags reader */
uint8_t* nodeGetSubmodFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].submod_flags;
}

/** Submodule flags writer */
void nodeSetSubmodFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].submod_flags = flags;
}

/** Router flags reader */
uint8_t* nodeGetRouterFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].router_flags;
}

/** Router flags writer */
void nodeSetRouterFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].router_flags = flags;
}

/** Return count of configured submodules */
uint8_t nodeGetSubModuleCount(void)
{
    uint8_t count = node.subModCnt;
    if (count > MAX_SUB_MODULES)
        count = MAX_SUB_MODULES;
    return count;
}


bool nodeIsValidSubmodule(uint8_t index)
{
    return (index < nodeGetSubModuleCount());
}

const personalityDef_t* nodeGetPersonality(uint8_t personalityIndex)
{
    if (personalityIndex >= runtimePersonalityCount)
        return NULL;

    return &runtimePersonalityTable[personalityIndex];
}


/** Pretty print nodeInfo_t */
/**
 * @brief Prints a pretty formatted version of the nodeInfo_t structure to the Serial console.
 *
 * @param node Pointer to the nodeInfo_t structure to print.
 */
void printNodeInfo(const nodeInfo_t* node) 
{
  Serial.println("\n--------- nodeInfo_t ---------");
  Serial.printf("NodeInfo:\n");
  Serial.printf("  nodeID: 0x%08X\n", node->nodeID);
  Serial.printf("  nodeTypeMsg: 0x%03X\n", node->nodeTypeMsg);
  Serial.printf("  nodeTypeDLC: %d\n", node->nodeTypeDLC);
  Serial.printf("  subModCnt: %d\n", node->subModCnt);

  Serial.printf("  subModule array:\n");
  for (uint8_t i = 0; i < node->subModCnt; i++) {
    /** Pointer to the personality definition for this sub-module */
    const personalityDef_t* p = &runtimePersonalityTable[node->subModule[i].personalityIndex]; 

    Serial.printf("     subModule[%d]:\n", i);
    Serial.printf("       personalityId: %d\n", node->subModule[i].personalityId);
    Serial.printf("       personalityIndex: %d\n", node->subModule[i].personalityIndex);
    Serial.printf("(p->)    dataMsgId: 0x%03X\n", p->dataMsgId);
    Serial.printf("(p->)    dataMsgDlc: %d\n", p->dataMsgDlc);
    Serial.printf("(p->)    gpio Pin: %d\n", p->gpioPin);
    if (node->subModule[i].personalityId == PERS_GPIO_INPUT) {
        Serial.printf("       gpio_input flags: 0x%02X\n", node->subModule[i].config.gpioInput.flags); 
        Serial.printf("       gpio_input debounce_ms: %d\n", node->subModule[i].config.gpioInput.debounce_ms); 
        Serial.printf("       gpio_input reserved: 0x%02X\n", node->subModule[i].config.gpioInput.reserved); 
    } else {
        Serial.printf("       config bytes 0x%02X 0x%02X 0x%02X\n", node->subModule[i].config.rawConfig[0], node->subModule[i].config.rawConfig[1], node->subModule[i].config.rawConfig[2]);
    }
    Serial.printf("       introMsgId: 0x%03X\n", node->subModule[i].introMsgId);
    Serial.printf("       introMsgDLC: %d\n", node->subModule[i].introMsgDLC);
    Serial.printf("       submod_flags: 0x%02X\n", node->subModule[i].submod_flags);
    Serial.printf("       producer_flags: 0x%02X\n", node->subModule[i].producer_flags);
    Serial.printf("       router_flags: 0x%02X\n", node->subModule[i].router_flags);
    Serial.printf("       runTime:\n");
    Serial.printf("         last_change_ms: %d\n", node->subModule[i].runTime.last_change_ms);
    Serial.printf("         valueU32: %d\n", node->subModule[i].runTime.valueU32);
    Serial.printf("         kind: %d\n", node->subModule[i].runTime.kind);
    Serial.printf("         period_ms: %d\n", node->subModule[i].runTime.period_ms);
    Serial.printf("         last_published_value: %d\n", node->subModule[i].runTime.last_published_value);
    // You can continue this pattern for each field in the subModule_t structure
  }
  Serial.println("--------- nodeInfo_t ---------\n");

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

uint32_t getEpochTime() 
{
  /* Get time from the system clock and return it as a uint32_t */
  struct timespec newTime;

  clock_gettime(CLOCK_REALTIME, &newTime); /* Read time from ESP32 clock*/

  return (uint32_t)newTime.tv_sec;
}