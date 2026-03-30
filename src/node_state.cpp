#include "node_state.h"
#include "esp_log.h"

static const char *TAG = "node_state";

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
  ESP_LOGI(TAG, "[INIT] Struct Sizes - nodeInfo_t: %d, subModule_t: %d, runTime_t: %d",
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

    ESP_LOGI(TAG, "[INIT] Node ID extracted: 0x%02X%02X%02X%02X  (0x%08X)",
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
    ESP_LOGD("NODEINFO", "--------- nodeInfo_t ---------");
    ESP_LOGD("NODEINFO", "NodeInfo:");
    ESP_LOGD("NODEINFO", "  nodeID: 0x%08X", node->nodeID);
    ESP_LOGD("NODEINFO", "  nodeTypeMsg: 0x%03X", node->nodeTypeMsg);
    ESP_LOGD("NODEINFO", "  nodeTypeDLC: %d", node->nodeTypeDLC);
    ESP_LOGD("NODEINFO", "  subModCnt: %d", node->subModCnt);

    ESP_LOGD("NODEINFO", "  subModule array:");
    for (uint8_t i = 0; i < node->subModCnt; i++) {

        const personalityDef_t* p =
            &runtimePersonalityTable[node->subModule[i].personalityIndex];

        ESP_LOGD("NODEINFO", "     subModule[%d]:", i);
        ESP_LOGD("NODEINFO", "       personalityId: %d",
                 node->subModule[i].personalityId);
        ESP_LOGD("NODEINFO", "       personalityIndex: %d",
                 node->subModule[i].personalityIndex);

        ESP_LOGD("NODEINFO", "       (p->) dataMsgId: 0x%03X", p->dataMsgId);
        ESP_LOGD("NODEINFO", "       (p->) dataMsgDlc: %d", p->dataMsgDlc);
        ESP_LOGD("NODEINFO", "       (p->) gpio Pin: %d", p->gpioPin);

        if (node->subModule[i].personalityId == PERS_GPIO_INPUT) {
            ESP_LOGD("NODEINFO", "       gpio_input flags: 0x%02X",
                     node->subModule[i].config.gpioInput.flags);
            ESP_LOGD("NODEINFO", "       gpio_input debounce_ms: %d",
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
        ESP_LOGD("NODEINFO", "       introMsgDLC: %d",
                 node->subModule[i].introMsgDLC);
        ESP_LOGD("NODEINFO", "       submod_flags: 0x%02X",
                 node->subModule[i].submod_flags);
        ESP_LOGD("NODEINFO", "       producer_flags: 0x%02X",
                 node->subModule[i].producer_flags);
        ESP_LOGD("NODEINFO", "       router_flags: 0x%02X",
                 node->subModule[i].router_flags);

        ESP_LOGD("NODEINFO", "       runTime:");
        ESP_LOGD("NODEINFO", "         last_change_ms: %d",
                 node->subModule[i].runTime.last_change_ms);
        ESP_LOGD("NODEINFO", "         valueU32: %d",
                 node->subModule[i].runTime.valueU32);
        ESP_LOGD("NODEINFO", "         kind: %d",
                 node->subModule[i].runTime.kind);
        ESP_LOGD("NODEINFO", "         period_ms: %d",
                 node->subModule[i].runTime.period_ms);
        ESP_LOGD("NODEINFO", "         last_published_value: %d",
                 node->subModule[i].runTime.last_published_value);
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

uint32_t getEpochTime() 
{
  /* Get time from the system clock and return it as a uint32_t */
  struct timespec newTime;

  clock_gettime(CLOCK_REALTIME, &newTime); /* Read time from ESP32 clock*/

  return (uint32_t)newTime.tv_sec;
}