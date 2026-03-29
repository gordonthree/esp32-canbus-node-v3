#include "storage.h"           /* NVS storage routines */
#include "personality_table.h" /* Personality definitions */
#include "can_producer.h"      /* CAN producer definitions */ 

// Global mutex for NVS access (declared in main.cpp)
extern SemaphoreHandle_t flashMutex;

/** Set this flag when the loaded configuration is valid (CRC matches) */
static bool FLAG_VALID_CONFIG = false; 

/* ============================================================================
 *  HELPERS
 * ========================================================================== */

/** * @brief Prints a hex dump of a memory block to the Serial console.
 * @param ptr Pointer to the memory block
 * @param size Size of the block in bytes
 */
void printHexDump(const void* ptr, size_t size) {
    const uint8_t* p = (const uint8_t*)ptr;
    char buf[16]; /* Buffer for hex formatting */

    Serial.println("\n--- nodeInfo_t Hex Dump ---");
    for (size_t i = 0; i < size; i++) {
        /* Print address offset every 16 bytes */
        if (i % 16 == 0) {
            if (i > 0) Serial.println();
            Serial.printf("%04X: ", (uint16_t)i);
        }

        Serial.printf("%02X ", p[i]);
    }
    Serial.println("\n---------------------------");
}



/**
 * @brief Create a sanitized copy of a nodeInfo_t struct.
 * @details This function takes a nodeInfo_t struct as input and creates a new
 *          copy of the struct with all volatile fields such as
 *          lastSeen, valueU32, and last_published_value fields zeroed out.
 * @param src Pointer to the nodeInfo_t struct to be sanitized.
 * @return A sanitized copy of the input nodeInfo_t struct.
 */
nodeInfo_t makeSanitizedNodeInfo(const nodeInfo_t *src)
{
    /* copy the live nodeInfo_t struct */
    nodeInfo_t out = *src; 

    /* zero out the volatile fields */
    for (uint8_t i = 0; i < MAX_SUB_MODULES; i++) {
        out.subModule[i].lastSeen = 0;
        out.subModule[i].runTime.last_change_ms = 0;
        out.subModule[i].runTime.valueU32 = 0;
        out.subModule[i].runTime.last_published_value = 0;
        out.subModule[i].submod_flags &= ~SUBMOD_FLAG_DIRTY;  /**< strip runtime-only dirty flag */
    }

    return out;
}

/* ============================================================================
 *  ROUTE TABLE LOAD/SAVE
 * ========================================================================== */

void loadRoutesFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, true)) {
        /* Check version, return 0 if not found*/
        const uint8_t version = prefs.getUChar(ROUTE_VERSION, 0); 
        if (version != ROUTER_VERSION) {
            prefs.end();
            xSemaphoreGive(flashMutex);
            Serial.println("Route table version mismatch");
            return;
        }
        if (prefs.isKey(ROUTE_KEY)) 
            {
                prefs.getBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
                prefs.getBytes(ROUTE_CRC_KEY, &g_routesCrc, sizeof(g_routesCrc));
            }
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveRoutesToNVS()
{

    /* First loop through the array and calculate the CRC 
       and update the timestamp for each route entry */

    for (uint8_t i = 0; i < MAX_ROUTES; i++) {
        /* Pointer to CRC data struct */
        route_entry_crc_t *crc = &g_routesCrc[i]; 
        
        if (!crc->in_use) continue; /* skip unused entries */
        
        crc->ts  = getEpochTime();

        printf("[CRC] route %d CRC=0x%04X TS=%d in_use=%d\n",
           i, crc->crc, crc->ts, crc->in_use);

    }

    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, false)) {
        prefs.putUChar(ROUTE_VERSION, ROUTER_VERSION);                     /* Write version */
        prefs.putBytes(ROUTE_KEY, g_routes, sizeof(g_routes));             /* Write route table */
        prefs.putBytes(ROUTE_CRC_KEY, g_routesCrc, sizeof(g_routesCrc));   /* Write route table CRC */ 
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}


/* ============================================================================
 *  NODE CONFIG LOAD/SAVE
 * ========================================================================== */

 /**
  * @brief Load default subModule configuration from the static personality
  * library and initialize the node configuration.
  * 
  * Internal personalities represent node hardware (CPU temp, heap, etc.)
  * and must be instantiated as submodules at boot.
  * 
  */
 void loadNodeDefaults() {

    nodeInfo_t& node = *nodeGetInfo();

    /** Set the number of submodules */
    node.subModCnt = g_submodules_count;

    /* Guardrail: personality library must define at least one submodule */
    if (g_submodules_count == 0) {
        printf("[INIT] Error: loadNodeDefaults(): submod_setup is empty (count = 0)\n");
        return;
    }

    /* Guardrail: pointer should never be NULL, but check anyway */
    if (submod_setup == NULL) {
        printf("[INIT] Error: loadNodeDefaults(): submod_setup pointer is NULL\n");
        return;
    }

    /** Set the node type from personality library */
    node.nodeTypeDLC = g_personalityNode.nodeTypeDLC; 
    node.nodeTypeMsg = g_personalityNode.nodeTypeMsg;
    
    /** Copy hardware submodules from submod_setup array */
    for (uint8_t i = 0; i < g_submodules_count; i++) {
        node.subModule[i] = submod_setup[i];    /* copy user config from submod_setup */
        node.subModule[i].personalityIndex = i; /* assign matching index */
        node.subModule[i].personalityId =
            runtimePersonalityTable[i].personalityId; /* force submodule personalityId to match runtime table */
    }

    /** Add internal submodules from personality table */
    for (uint8_t i = 0; i < runtimePersonalityCount; i++) {

        const personalityDef_t *p = &runtimePersonalityTable[i];

        if (!(p->flags & BUILDER_FLAG_IS_INTERNAL))
            continue; /* skip non-internal personalities */

        if (node.subModCnt >= MAX_SUB_MODULES)
            break;    /* no more space for submodules */

        Serial.printf("[INIT] Adding internal sub-module: index %d, intro msg 0x%02X\n", i, p->introMsgId);

        /** Create a internal sub-module and zero it out */
        subModule_t *sub = &node.subModule[node.subModCnt++];
        memset(sub, 0, sizeof(*sub));

        
        sub->personalityId    = p->personalityId; /**< assign personality */
        sub->personalityIndex = i;                /**< assign index */ 

        /**  internal submodules usually have no user config */
        memset(sub->config.rawConfig, 0, sizeof(sub->config.rawConfig));

        // Use personality table for CAN reporting
        sub->introMsgId          = p->introMsgId;
        sub->introMsgDLC         = p->introMsgDlc;

        // Mark as internal
        sub->submod_flags       |= SUBMOD_FLAG_INTERNAL;
        sub->producer_flags     |= PRODUCER_FLAG_ENABLED;

        // Producer defaults
        sub->runTime.kind        = PRODUCER_KIND_PERIODIC;
        sub->runTime.period_ms   = p->period_ms;             /* 10 seconds */
    }


    Serial.printf("[INIT] Defaults loaded, submod count: %d\n", node.subModCnt);

    if (runtimePersonalityCount < g_submodules_count) {
       Serial.println("[INIT] WARNING: runtimePersonalityCount less than g_submodules_count!");
    }

    // printNodeInfo(&node);
}



/**
 * @brief Handles the erase NVS command from the master node
 *
 * This function attempts to erase the NVS configuration and
 * reset the node to its default configuration. If the erase
 * operation fails due to a mutex timeout, the function will
 * retry up to 3 times with a short delay in between retries.
 */
void handleEraseCfgNVS() 
{
  ConfigStatus cfgStatus;
  int retries = 0;

  Serial.println("\nErasing config...");

  do {
      cfgStatus = eraseConfigNvs();

      if (cfgStatus == CFG_OK) {
          Serial.println("Config erased successfully, rebooting...");
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before reboot */
          ESP.restart(); /**< Reboot the ESP32 */
          break;
      }

      if (cfgStatus == CFG_ERR_NOT_FOUND) {
          Serial.println("Config not found.");
          break;
      }

      if (cfgStatus == CFG_ERR_MUTEX) {
          Serial.printf("Flash busy - Retry %d/3...\n", retries + 1);
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
      }

  } while ((cfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (cfgStatus == CFG_ERR_MUTEX) {
      Serial.println("Critical Error: Could not access NVS (Mutex Timeout)");
  }
}


/**
 * @brief Erases the node configuration and CRC from NVS.
 * @return ConfigStatus status of the erase operation (OK, NOT_FOUND, or MUTEX).
 */
ConfigStatus eraseConfigNvs() 
{
  /* Attempt to acquire the flash mutex */
  if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return CFG_ERR_MUTEX;
  }

  Preferences prefs;
  if (!prefs.begin("node_cfg", false)) {
    xSemaphoreGive(flashMutex);
    return CFG_ERR_NVS_OPEN;
  }

  /* Check if the data key exists before attempting removal */
  if (!prefs.isKey("node_data")) {
    prefs.end();
    xSemaphoreGive(flashMutex);
    return CFG_ERR_NOT_FOUND;
  }

  prefs.remove("node_data");
  prefs.remove("node_crc");

  prefs.end();
  xSemaphoreGive(flashMutex);

  return CFG_OK;
}

/**
 * @brief Saves the node configuration to NVS and updates the external CRC.
 * @param node Reference to the nodeInfo_t struct to persist.
 * @return CFG_OK on success, or CFG_ERR_MUTEX if flash is busy.
 */
ConfigStatus saveConfigNvs() 
{
    /* record start time */
    const uint64_t startTime = (uint64_t)esp_timer_get_time();

    /* get a clean copy of the nodeInfo_t struct with volatile fields zeroed out */
    const nodeInfo_t sanitizedNode = makeSanitizedNodeInfo(nodeGetInfo());

    /* hash the sanitized nodeInfo_t struct to get the CRC key */
    const uint16_t crc = crc16_ccitt((const uint8_t*)&sanitizedNode, sizeof(sanitizedNode));

    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    /* Open namespace in read/write mode */
    if (!prefs.begin(NODE_NS, false)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NVS_OPEN; /* Return error if NVS open fails */
    }

    /* Write the version, crc and configuration blob to NVS */
    prefs.putUChar(NODE_VERSION, NODEINFO_VERSION);
    prefs.putUShort(NODE_CRC_KEY, crc);
    prefs.putBytes(NODE_DATA_KEY, &sanitizedNode, sizeof(sanitizedNode));

    /* Close NVS */
    prefs.end();

    /* Release the flash mutex */
    xSemaphoreGive(flashMutex);

    /* record elapsed time */
    const uint64_t elapsedTime = (uint64_t)esp_timer_get_time() - startTime;

    /* print elapsed time */
    Serial.printf("[NVS] Save complete. Elapsed time: %lu us\n", elapsedTime);

    return CFG_OK;
}


/**
 * @brief Loads the current node configuration and CRC from NVS.
 * @param[in,out] node Reference to the nodeInfo_t struct to be loaded.
 * @return ConfigStatus indicating the result of the operation.
 * @retval CFG_OK Load successful and CRC valid.
 * @retval CFG_ERR_MUTEX Failed to acquire flash mutex.
 * @retval CFG_ERR_NOT_FOUND No configuration exists in NVS.
 * @retval CFG_ERR_CRC Data found but CRC is invalid (corrupt).
 */
ConfigStatus loadConfigNvs(nodeInfo_t& node)
{
    
    /* Temporary storage for NVS data before CRC validation */
    nodeInfo_t quarantineNode = {0}; 

    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    /* Open namespace in read-only mode */
    Preferences prefs;
    if (!prefs.begin(NODE_NS, true)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /* Check version, return 0 if not found*/
    const uint8_t version = prefs.getUChar(NODE_VERSION, 0);

    /* Check version, return error if not found*/
    if (version != NODEINFO_VERSION) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_VERS_ERROR;
    }

    /* Check if the data key exists */
    if (!prefs.isKey(NODE_DATA_KEY)) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /* Check if the CRC key exists */
    if (!prefs.isKey(NODE_CRC_KEY)) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_CRC_MISS;
    }

    /* Read data and CRC from NVS */
    prefs.getBytes(NODE_DATA_KEY, &quarantineNode, sizeof(quarantineNode));
    uint16_t storedCrc = prefs.getUShort(NODE_CRC_KEY, 0);

    prefs.end();
    xSemaphoreGive(flashMutex);

    /* Sanitize data before CRC validation */
    const nodeInfo_t sanitized = makeSanitizedNodeInfo(&quarantineNode);

    /* Calculate CRC of sanitized data */
    const uint16_t crc = crc16_ccitt((const uint8_t*)&sanitized, sizeof(sanitized));

    /* Validate CRC */
    if ((crc != storedCrc)) {
        Serial.printf("[INIT] CRC mismatch: %d != %d\n", crc, storedCrc);    

        return CFG_ERR_CRC;
    }

    /* CRC passed, copy data to live node struct */
    memcpy(&node, &sanitized, sizeof(node));

    return CFG_OK;
}

/* ============================================================================
 *  SUBMODULE MANAGEMENT API
 * ========================================================================== */

 int addSubmodule(const uint8_t personalityId,
                 const uint8_t* configBytes,
                 size_t configLength)
{
    nodeInfo_t& node = *nodeGetInfo();

    /* Bounds check: personalityId must exist */
    if (personalityId >= PERSONALITY_MAX) {
        return -1;
    }

    /* Bounds check: no room left */
    if (node.subModCnt >= MAX_SUB_MODULES) {
        return -1;
    }

    /* Find the first empty slot */
    int index = -1;
    for (int i = 0; i < MAX_SUB_MODULES; i++) {
        if (node.subModule[i].personalityIndex == 0xFF) {  // or however "empty" is defined
            index = i;
            break;
        }
    }

    if (index < 0) {
        return -1;  // no free slot
    }

    /* Retrieve the personality template */
    const personalityDef_t* p = &templateTable[personalityId];

    /* Initialize the new submodule */
    subModule_t* sub = &node.subModule[index];
    memset(sub, 0, sizeof(subModule_t));

    /* Set the personality index equal to the template id */
    sub->personalityIndex = personalityId;

    /* Copy rawConfig (3 bytes max) */
    size_t copyLen = (configLength > 3) ? 3 : configLength;
    memcpy(sub->config.rawConfig, configBytes, copyLen);

    /* Initialize network fields if CAP_NETWORK is set */
    if (p->capabilities & CAP_NETWORK) {
        /* Expecting configBytes to contain a 32-bit nodeID */
        if (configLength >= 4) {
            sub->networkNodeId =
                ((uint32_t)configBytes[0] << 24) |
                ((uint32_t)configBytes[1] << 16) |
                ((uint32_t)configBytes[2] << 8)  |
                ((uint32_t)configBytes[3]);
        }

        sub->lastSeen = 0; /* zero timestamp */
        memset(sub->netConfig, 0, sizeof(sub->netConfig)); /* clear network config */
    }

    /* Mark runtime flags */
    sub->submod_flags = SUBMOD_FLAG_DIRTY;

    /* Increment count */
    node.subModCnt++;


    return index;
}


/**
 * @brief Attempts to load the configuration from NVS.
 *
 * This function attempts to load the configuration from NVS and
 * initializes the hardware if successful. If the configuration
 * is invalid or not found, it loads the default configuration from
 * the build flag node type and starts in PROVISIONING MODE. If the
 * function is unable to access NVS due to a mutex timeout, it
 * loads the default configuration from the build flag node type.
 */
void handleReadCfgNVS() 
{
  ConfigStatus loadCfgStatus;
  int retries = 0;
  /* Record start time */
  const uint64_t startTime = (uint64_t)esp_timer_get_time();

  Serial.println("[INIT] Loading config from NVS...");
  nodeInfo_t& node = *nodeGetInfo();

  do {
    loadCfgStatus = loadConfigNvs(node);

    if (loadCfgStatus == CFG_OK) {
      Serial.println("[INIT] Config loaded successfully.");
      FLAG_VALID_CONFIG = true;
      break;
    }

    if (loadCfgStatus = CFG_VERS_ERROR) {
      Serial.println("[INIT] Config version mismatch - NVS load aborted.");
      break;
    }

    if (loadCfgStatus == CFG_ERR_CRC) {
      Serial.println("[INIT] Config CRC mismatch - NVS load aborted.");
      break;
    }

    if (loadCfgStatus == CFG_ERR_CRC_MISS) {
      Serial.println("[INIT] Config CRC not found in NVS - NVS load aborted.");
      break;
    }

    if (loadCfgStatus == CFG_ERR_NOT_FOUND) {
      Serial.println("[INIT] Config data not found in NVS - NVS load aborted.");
      break;
    }

    if (loadCfgStatus == CFG_ERR_MUTEX) {
      Serial.printf("[INIT] Flash busy - Retry %d/3...\n", retries + 1);
      vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
    }

  } while ((loadCfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (loadCfgStatus == CFG_ERR_MUTEX) {
    Serial.println("[INIT] Flash busy, mutex timeout - NVS load aborted.");
  }

  const uint64_t loadTime = (uint64_t)esp_timer_get_time() - startTime;
  Serial.printf("[INIT] Config load took %d ms\n", (loadTime / 1000));
}

bool removeSubmodule(const uint8_t index)
{
    if (index >= MAX_SUB_MODULES) {
        return false;
    }

    nodeInfo_t& node = *nodeGetInfo();

    /* If the slot is already empty, nothing to do */
    if (node.subModule[index].personalityIndex == 0xFF) {
        return false;
    }

    /* Shift remaining submodules down to keep table contiguous */
    for (int i = index; i < (node.subModCnt - 1); i++) {
        node.subModule[i] = node.subModule[i + 1];
    }

    /* Clear the last entry */
    memset(&node.subModule[node.subModCnt - 1], 0xFF, sizeof(subModule_t));

    /* Decrement count */
    node.subModCnt--;

    /* Persist to NVS */
    // saveSubmodulesToNVS();

    return true;
}

bool nodeGetValidConfig(void)
{
    return FLAG_VALID_CONFIG;
}
