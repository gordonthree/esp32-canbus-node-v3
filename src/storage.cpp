#include "storage.h"           /* NVS storage routines */
#include "personality_table.h" /* Personality definitions */
#include "can_producer.h"      /* CAN producer definitions */ 
// #include "isr_gpio.h"          /* GPIO interrupt routines */

// Global mutex for NVS access (declared in main.cpp)
extern SemaphoreHandle_t flashMutex;

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


/* ============================================================================
 *  ROUTE TABLE LOAD/SAVE
 * ========================================================================== */

void loadRoutesFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, true)) {
        if (prefs.isKey(ROUTE_KEY))
            prefs.getBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveRoutesToNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, false)) {
        prefs.putBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

/* ============================================================================
 *  PRODUCER CONFIG LOAD/SAVE
 * ========================================================================== */

void loadProducerCfgFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(PROD_NS, true)) {

        if (prefs.isKey(PROD_KEY)) 
        {
            // Read from NVS

            runTime_t temp[MAX_SUB_MODULES] = {0}; /* zero-initialize array */
            size_t expected = sizeof(temp);

            size_t read = prefs.getBytes(PROD_KEY, temp, expected);

            if (read == expected) {
                // Copy into submodules
                for (uint8_t i = 0; i < MAX_SUB_MODULES; i++) {
                    node.subModule[i].runTime = temp[i];
                }
            }
        }

        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveProducerCfgToNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(PROD_NS, false)) {

        runTime_t temp[MAX_SUB_MODULES];

        // Copy from submodules
        for (uint8_t i = 0; i < MAX_SUB_MODULES; i++) {
            temp[i] = node.subModule[i].runTime;
        }

        prefs.putBytes(PROD_KEY, temp, sizeof(temp));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void deleteProducerCfgFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(PROD_NS, false)) {

        if (prefs.isKey(PROD_KEY)) {
            prefs.remove(PROD_KEY);   // <-- delete the stored producer_cfg array
        }

        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

/* ============================================================================
 *  NODE CONFIG LOAD/SAVE
 * ========================================================================== */

 /**
  * @brief Load default subModule configuration from personality library if no config
  * was found in NVS.
  * 
  */
 void loadNodeDefaults() {

    /** Set the number of submodules */
    node.subModCnt = g_submodules_count;

    /* Guardrail: personality library must define at least one submodule */
    if (g_submodules_count == 0) {
        printf("[ERR] loadNodeDefaults(): submod_setup is empty (count = 0)\n");
        return;
    }

    /* Guardrail: pointer should never be NULL, but check anyway */
    if (submod_setup == NULL) {
        printf("[ERR] loadNodeDefaults(): submod_setup pointer is NULL\n");
        return;
    }

    /** Set the node type from personality library */
    node.nodeTypeDLC = g_personalityNode.nodeTypeDLC; 
    node.nodeTypeMsg = g_personalityNode.nodeTypeMsg;
    
    /** Copy hardware submodules from submod_setup array */
    for (uint8_t i = 0; i < g_submodules_count; i++) {
        node.subModule[i] = submod_setup[i];    /* copy user config from submod_setup */
        node.subModule[i].personalityIndex = i; /* assign matching index */
    }

    /** Add virtual submodules from personality table */
    for (uint8_t i = 0; i < g_personalityCount; i++) {

        const personalityDef_t *p = &g_personalityTable[i];

        if (!(p->flags & BUILDER_FLAG_IS_VIRTUAL))
            continue; /* skip non-virtual personalities */

        if (node.subModCnt >= MAX_SUB_MODULES)
            break;    /* no more space for submodules */

        Serial.printf("[INIT] Adding virtual sub-module: index %d, intro msg 0x%02X\n", i, p->introMsgId);

        /** Create a virtual sub-module and zero it out */
        subModule_t *sub = &node.subModule[node.subModCnt++];
        memset(sub, 0, sizeof(*sub));

        
        sub->personalityId    = p->personalityId; /**< assign personality */
        sub->personalityIndex = i;                /**< assign index */ 

        /**  Virtual submodules usually have no user config */
        memset(sub->config.rawConfig, 0, sizeof(sub->config.rawConfig));

        // Use personality table for CAN reporting
        sub->introMsgId          = p->introMsgId;
        sub->introMsgDLC         = p->introMsgDlc;

        // Mark as virtual + read-only
        sub->submod_flags       |= SUBMOD_FLAG_VIRTUAL;
        sub->producer_flags     |= PRODUCER_FLAG_ENABLED;

        // Producer defaults
        sub->runTime.kind        = PRODUCER_KIND_PERIODIC;
        sub->runTime.period_ms   = p->period_ms;             /* 10 seconds */
    }


    Serial.printf("[INIT] Defaults loaded, submod count: %d\n", g_submodules_count);

    printNodeInfo(&node);

    /** Print hex dump of nodeInfo_t */
    // printHexDump(&node, sizeof(node));
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

  Serial.println("\n[INIT] Loading config from NVS...");

  do {
      loadCfgStatus = loadConfigNvs(node);

      if (loadCfgStatus == CFG_OK) {
          Serial.println("[INIT] Config loaded successfully.");
          FLAG_VALID_CONFIG = true;
        //   initHardware(); /**< Initialize the hardware */
          break;
      }

      if (loadCfgStatus == CFG_ERR_CRC) {
          Serial.println("[INIT] Config CRC mismatch - loading internal defaults");
          loadNodeDefaults(); /**< load defaults from build flag node type */
          break;
      }

      if (loadCfgStatus == CFG_ERR_CRC_MISS) {
          Serial.println("[INIT] Config CRC not found in NVS - loading internal defaults");
          loadNodeDefaults(); /**< load defaults from build flag node type */
          break;
      }

      if (loadCfgStatus == CFG_ERR_NOT_FOUND) {
          Serial.println("[INIT] Config data not found in NVS - loading internal defaults");
          loadNodeDefaults(); /**< load defaults from build flag node type */
          break;
      }

      if (loadCfgStatus == CFG_ERR_MUTEX) {
          Serial.printf("[INIT] Flash busy - Retry %d/3...\n", retries + 1);
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
      }

  } while ((loadCfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (loadCfgStatus == CFG_ERR_MUTEX) {
      Serial.println("[INIT] Critical Error: Could not access NVS (Mutex Timeout), loading internal defaults.");
      loadNodeDefaults(); /* load defaults from build flag node type */
  }
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
ConfigStatus saveConfigNvs(const nodeInfo_t& node) 
{
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

    /** Calculate the CRC of the current RAM buffer before saving */
    const uint16_t currentCrc = getConfigurationCRC(node);

    /** Write the configuration blob and the CRC key separately */
    prefs.putBytes(NODE_DATA_KEY, &node, sizeof(nodeInfo_t));
    prefs.putUShort(NODE_CRC_KEY, currentCrc);

    prefs.end();
    xSemaphoreGive(flashMutex);

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
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    if (!prefs.begin(NODE_NS, true)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /** Check if the data key exists */
    if (!prefs.isKey(NODE_DATA_KEY)) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /** Check if the CRC key exists */
    if (!prefs.isKey(NODE_CRC_KEY)) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_CRC_MISS;
    }

    // Read data and stored CRC
    prefs.getBytes(NODE_DATA_KEY, &node, sizeof(nodeInfo_t));
    uint16_t storedCrc = prefs.getUShort(NODE_CRC_KEY, 0);

    prefs.end();
    xSemaphoreGive(flashMutex);

    // Validate
    if (getConfigurationCRC(node) != storedCrc) {
        Serial.printf("[INIT] CRC mismatch: %d != %d\n", getConfigurationCRC(node), storedCrc);    

        return CFG_ERR_CRC;
    }

    return CFG_OK;
}

/**
 * @brief Saves a subModule_t to NVS using the Preferences library.
 * @param subModule Pointer to the subModule_t structure to save.
 * @param index Index of the subModule_t in the array.
 * @return CFG_OK on success, or CFG_ERR_MUTEX if flash is busy.
 */
ConfigStatus saveSubModuleNvs(const subModule_t& subModule, uint8_t index)
{
    char data_key[16];
    char crc_key[16];

    /** Calculate the CRC of the current RAM buffer before saving */
    const uint16_t currentCrc = getSubModuleCRC(subModule); 

    /** Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    if (!prefs.begin(NODE_NS, false)) {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NVS_OPEN; /* Return error if NVS open fails */
    }

    /* Generate unique keys */
    snprintf(data_key, sizeof(data_key), "subModule_%d", index);
    snprintf(crc_key, sizeof(crc_key), "sub_%d_crc", index);

    /** Write the configuration blob and the CRC key separately */
    prefs.putBytes(data_key, &subModule, sizeof(subModule_t));
    prefs.putUShort(crc_key, currentCrc);

    /** Close NVS and release the mutex */
    prefs.end();
    xSemaphoreGive(flashMutex);

    return CFG_OK;
}