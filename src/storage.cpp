#include "storage.h"

// Global mutex for NVS access (declared in main.cpp)
extern SemaphoreHandle_t flashMutex;



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

  Serial.println("\nLoading config...");

  do {
      loadCfgStatus = loadConfigNvs(node);

      if (loadCfgStatus == CFG_OK) {
          Serial.println("Config loaded successfully.");
          FLAG_VALID_CONFIG = true;
          initHardware(); /**< Initialize the hardware */
          break;
      }

      if (loadCfgStatus == CFG_ERR_CRC || loadCfgStatus == CFG_ERR_NOT_FOUND) {
          Serial.println("Invalid config - Starting in PROVISIONING MODE");
          loadDefaults(NODEMSGID); /**< load defaults from build flag node type */
          break;
      }

      if (loadCfgStatus == CFG_ERR_MUTEX) {
          Serial.printf("Flash busy - Retry %d/3...\n", retries + 1);
          vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
      }

  } while ((loadCfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

  if (loadCfgStatus == CFG_ERR_MUTEX) {
      Serial.println("Critical Error: Could not access NVS (Mutex Timeout)");
      loadDefaults(NODEMSGID); /* load defaults from build flag node type */
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
    uint16_t currentCrc = getConfigurationCRC(node);

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

    // Check if the key exists before reading
    if (!prefs.isKey(NODE_DATA_KEY)) {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    // Read data and stored CRC
    prefs.getBytes(NODE_DATA_KEY, &node, sizeof(nodeInfo_t));
    uint16_t storedCrc = prefs.getUShort(NODE_CRC_KEY, 0);

    prefs.end();
    xSemaphoreGive(flashMutex);

    // Validate
    if (getConfigurationCRC(node) != storedCrc) {
        return CFG_ERR_CRC;
    }

    return CFG_OK;
}
