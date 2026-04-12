#include "storage.h"           /* NVS storage routines */
#include "personality_table.h" /* Personality definitions */
#include "can_producer.h"      /* CAN producer definitions */
#include "esp_log.h"

static const char *TAG = "storage";
static const char *TAG_HEX = "HEX DUMP";

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
void printHexDump(const void *ptr, size_t size)
{
    const uint8_t *p = (const uint8_t *)ptr;

    ESP_LOGD(TAG_HEX, "--- nodeInfo_t Hex Dump ---");

    /* One row buffer: "0000: " + 16 * "FF " + null = ~60 bytes */
    char line[80];

    for (size_t i = 0; i < size; i += 16)
    {

        int pos = 0;

        /* Write the offset */
        pos += snprintf(&line[pos], sizeof(line) - pos,
                        "%04X: ", (uint16_t)i);

        /* Write up to 16 bytes */
        for (size_t j = 0; j < 16 && (i + j) < size; j++)
        {
            pos += snprintf(&line[pos], sizeof(line) - pos,
                            "%02X ", p[i + j]);
            if (pos >= sizeof(line))
                break;
        }

        ESP_LOGD(TAG_HEX, "%s", line);
    }

    ESP_LOGD(TAG_HEX, "---------------------------");
}


/**
 * @brief Create a sanitized copy of a nodeInfo_t struct.
 * @details This function takes a nodeInfo_t struct as input and creates a new
 *          copy of the struct with all volatile fields zeroed out. The function
 *          also removes internal submodules. Finally, the function recomputes
 *          the subModCnt field from scratch.
 * @param src Pointer to the nodeInfo_t struct to be sanitized.
 * @return A sanitized copy of the input nodeInfo_t struct.
 */
nodeInfo_t makeSanitizedNodeInfo(const nodeInfo_t *src)
{
    /* copy the provided nodeInfo_t struct into working buffer */
    nodeInfo_t out = *src;

    /* loop through submodules */ 
    for (uint8_t i = 0; i < MAX_SUB_MODULES; i++)
    {
        /* retrieve the personality index */
        const uint8_t persIdx =
            src->subModule[i].personalityIndex;

        /* Skip unused entries */
        if (persIdx == 0xFF) /* 0xFF indicates unused */
            continue;

        /* Get the personality definition */
        const personalityDef_t *p =
            nodeGetPersonality(persIdx);

        /* for internal and network submodules,clear the entire
           struct with 0xFF to indicate unused */
        if (p->flags & ( BUILDER_FLAG_IS_INTERNAL ))
        {
            memset(&out.subModule[i], 0xFF, sizeof(subModule_t));
            continue; /* skip to the next submodule */
        }

        /* zero out the volatile fields */
        out.subModule[i].lastSeen = 0;
        /* strip runtime-only dirty flag */
        out.subModule[i].submod_flags &= ~SUBMOD_FLAG_DIRTY;
        /* zero out the entire runTime struct */
        // memset(&out.subModule[i].runTime, 0, sizeof(runTime_t));
    }

    /* Recompute subModCnt from scratch */
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_SUB_MODULES; i++)
    {
        if (out.subModule[i].personalityIndex != 0xFF)
            count++;
    }
    out.subModCnt = count;

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
    if (prefs.begin(ROUTE_NS, true))
    {
        /* Check version, return 0 if not found*/
        const uint8_t version = prefs.getUChar(ROUTE_VERSION, 0);
        if (version != ROUTER_VERSION)
        {
            prefs.end();
            xSemaphoreGive(flashMutex);
            ESP_LOGW(TAG, "[NVS] Warning: Route table version mismatch");
            return;
        }
        /* Load the route table */
        if (prefs.isKey(ROUTE_KEY))
        {
            prefs.getBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
        }

        /* Load CRC data for the route table */
        if (prefs.isKey(ROUTE_CRC_KEY))
        {
            route_entry_crc_t *routeCrc = &g_routeCrc[0];
            prefs.getBytes(ROUTE_CRC_KEY, &g_routeCrc, sizeof(g_routeCrc));
        }
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveRoutesToNVS()
{

    /* First loop through the array and calculate the CRC
       and update the timestamp for each route entry */

    for (uint8_t i = 0; i < MAX_ROUTES; i++)
    {
        /* Pointer to CRC data struct */
        route_entry_crc_t *routeCrc = &g_routeCrc[i];

        const bool inUse = routerIsRouteInUse(i);

        if (!inUse)
            continue; /* skip unused entries */

        routeCrc->ts = getEpochTime();

        ESP_LOGD(TAG, "[CRC] route %u CRC=0x%04X TS=%lu in_use=%u",
                 i, routeCrc->crc, routeCrc->ts, inUse);
    }

    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, false))
    {
        prefs.putUChar(ROUTE_VERSION, ROUTER_VERSION);                   /* Write version */
        prefs.putBytes(ROUTE_KEY, g_routes, sizeof(g_routes));           /* Write route table */
        prefs.putBytes(ROUTE_CRC_KEY, g_routeCrc, sizeof(g_routeCrc));   /* Write route table CRC */ 
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
 * @details This function loads physical hardware submodules from the hardware 
 * personality table. This table remains the authoritative definition of the 
 * node's physical capabilities (GPIOs, ADCs, PWM, sensors, etc.).
 *
 * Older hardware tables may also include internal (non-physical) personalities.
 * Support for internal personalities in the hardware table is deprecated, but
 * retained for backward compatibility. New internal submodules should be added
 * via the discovery system instead.
 * 
 * See hardware_init.h and personality_template.h for details.
 *
 */
void loadNodeDefaults()
{

    nodeInfo_t &node = *nodeGetInfo();

    /** Set the number of submodules */
    node.subModCnt = g_submodules_count;

    /* Guardrail: personality library must define at least one submodule */
    if (g_submodules_count == 0)
    {
        ESP_LOGE(TAG, "[INIT] Error: loadNodeDefaults(): submod_setup is empty (count = 0)");
        return;
    }

    /* Guardrail: pointer should never be NULL, but check anyway */
    if (submod_setup == NULL)
    {
        ESP_LOGE(TAG, "[INIT] Error: loadNodeDefaults(): submod_setup pointer is NULL");
        return;
    }

    /** Set the node type from personality library */
    node.nodeTypeDLC = g_personalityNode.nodeTypeDLC;
    node.nodeTypeMsg = g_personalityNode.nodeTypeMsg;

    /** Copy hardware submodules from submod_setup array */
    for (uint8_t i = 0; i < g_submodules_count; i++)
    {
        node.subModule[i] = submod_setup[i];    /* copy user config from submod_setup */
        node.subModule[i].personalityIndex = i; /* assign matching index */
        node.subModule[i].personalityId =
            runtimePersonalityTable[i].personalityId; /* force submodule personalityId to match runtime table */
    }

    /** Add internal submodules from personality table */
    for (uint8_t i = 0; i < runtimePersonalityCount; i++)
    {
        /* @note: bootstrapping - this is the proper accessor */
        const personalityDef_t *p = &runtimePersonalityTable[i];

        if (!(p->flags & BUILDER_FLAG_IS_INTERNAL))
            continue; /* skip non-internal personalities */

        if (node.subModCnt >= MAX_SUB_MODULES)
            break; /* no more space for submodules */

        ESP_LOGI(TAG, "[INIT] Adding internal sub-module: index %d, intro msg 0x%02X", i, p->introMsgId);

        /** Create a internal sub-module and zero it out */
        subModule_t *sub = &node.subModule[node.subModCnt++];
        memset(sub, 0, sizeof(*sub));

        sub->personalityId = p->personalityId; /**< assign personality */
        sub->personalityIndex = i;             /**< assign index */

        /**  internal submodules usually have no user config */
        memset(sub->config.rawConfig, 0, sizeof(sub->config.rawConfig));

        // Use personality table for CAN reporting
        sub->introMsgId = p->introMsgId;
        sub->introMsgDLC = p->introMsgDlc;

        // Mark as internal
        sub->submod_flags |= SUBMOD_FLAG_INTERNAL;

        // Wiring into Producer and enable publishing
        sub->producer_flags |= PRODUCER_FLAG_PUBLISH_ENABLED | PRODUCER_FLAG_ACTIVE;

        // Producer defaults
        sub->producer_kind = PRODUCER_KIND_PERIODIC;
        sub->producer_period_ms = p->period_ms; /* 10 seconds */
    }

    ESP_LOGI(TAG, "[INIT] Defaults loaded, submod count: %d", node.subModCnt);

    if (runtimePersonalityCount < g_submodules_count)
    {
        ESP_LOGW(TAG, "[INIT] WARNING: runtimePersonalityCount less than g_submodules_count!");
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

    ESP_LOGI(TAG, "[NVS] Erasing config...");

    do
    {
        cfgStatus = eraseConfigNvs();

        if (cfgStatus == CFG_OK)
        {
            ESP_LOGI(TAG, "[NVS] Config erased successfully, rebooting...");
            vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before reboot */
            ESP.restart();                  /**< Reboot the ESP32 */
            break;
        }

        if (cfgStatus == CFG_ERR_NOT_FOUND)
        {
            ESP_LOGI(TAG, "[NVS] Erase: Config not found.");
            break;
        }

        if (cfgStatus == CFG_ERR_MUTEX)
        {
            ESP_LOGI(TAG, "[NVS] Erase: Flash busy - Retry %d/3...", retries + 1);
            vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
        }

    } while ((cfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

    if (cfgStatus == CFG_ERR_MUTEX)
    {
        ESP_LOGE(TAG, "[NVS] Critical Error: Could not access NVS (Mutex Timeout)");
    }
}

/**
 * @brief Erases the node configuration and CRC from NVS.
 * @return ConfigStatus status of the erase operation (OK, NOT_FOUND, or MUTEX).
 */
ConfigStatus eraseConfigNvs()
{
    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    if (!prefs.begin("node_cfg", false))
    {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NVS_OPEN;
    }

    /* Check if the data key exists before attempting removal */
    if (!prefs.isKey("node_data"))
    {
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
    const uint16_t crc = crc16_ccitt((const uint8_t *)&sanitizedNode, sizeof(sanitizedNode));

    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        return CFG_ERR_MUTEX;
    }

    Preferences prefs;
    /* Open namespace in read/write mode */
    if (!prefs.begin(NODE_NS, false))
    {
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
    ESP_LOGI(TAG, "[NVS] Save complete. Elapsed time: %lu us", elapsedTime);

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
ConfigStatus loadConfigNvs(nodeInfo_t &node)
{

    /* Temporary storage for NVS data before CRC validation */
    nodeInfo_t quarantineNode = {0};

    /* Attempt to acquire the flash mutex */
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        return CFG_ERR_MUTEX;
    }

    /* Open namespace in read-only mode */
    Preferences prefs;
    if (!prefs.begin(NODE_NS, true))
    {
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /* Check version, return 0 if not found*/
    const uint8_t version = prefs.getUChar(NODE_VERSION, 0);

    /* Check version, return error if not found*/
    if (version != NODEINFO_VERSION)
    {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_VERS_ERROR;
    }

    /* Check if the data key exists */
    if (!prefs.isKey(NODE_DATA_KEY))
    {
        prefs.end();
        xSemaphoreGive(flashMutex);
        return CFG_ERR_NOT_FOUND;
    }

    /* Check if the CRC key exists */
    if (!prefs.isKey(NODE_CRC_KEY))
    {
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
    const uint16_t crc = crc16_ccitt((const uint8_t *)&sanitized, sizeof(sanitized));

    /* Validate CRC */
    if ((crc != storedCrc))
    {
        ESP_LOGD(TAG, "[INIT] CRC mismatch: %d != %d", crc, storedCrc);

        return CFG_ERR_CRC;
    }

    /* CRC passed, copy data to live node struct */
    memcpy(&node, &sanitized, sizeof(node));

    return CFG_OK;
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

    ESP_LOGI(TAG, "[NVS] Loading config from NVS...");
    nodeInfo_t &node = *nodeGetInfo();

    do
    {
        loadCfgStatus = loadConfigNvs(node);

        if (loadCfgStatus == CFG_OK)
        {
            ESP_LOGI(TAG, "[NVS] Config loaded successfully.");
            FLAG_VALID_CONFIG = true;
            break;
        }

        if (loadCfgStatus = CFG_VERS_ERROR)
        {
            ESP_LOGW(TAG, "[NVS] Config version mismatch - NVS load aborted.");
            break;
        }

        if (loadCfgStatus == CFG_ERR_CRC)
        {
            ESP_LOGW(TAG, "[NVS] Config CRC mismatch - NVS load aborted.");
            break;
        }

        if (loadCfgStatus == CFG_ERR_CRC_MISS)
        {
            ESP_LOGW(TAG, "[NVS] Config CRC not found in NVS - NVS load aborted.");
            break;
        }

        if (loadCfgStatus == CFG_ERR_NOT_FOUND)
        {
            ESP_LOGW(TAG, "[NVS] Config data not found in NVS - NVS load aborted.");
            break;
        }

        if (loadCfgStatus == CFG_ERR_MUTEX)
        {
            ESP_LOGW(TAG, "[NVS] Flash busy - Retry %d/3...", retries + 1);
            vTaskDelay(pdMS_TO_TICKS(100)); /* Short sleep before retry */
        }

    } while ((loadCfgStatus == CFG_ERR_MUTEX) && (retries++ < 3));

    if (loadCfgStatus == CFG_ERR_MUTEX)
    {
        ESP_LOGW(TAG, "[NVS] Flash busy, mutex timeout - NVS load aborted.");
    }

    const uint64_t loadTime = (uint64_t)esp_timer_get_time() - startTime;
    ESP_LOGI(TAG, "[NVS] Config load took %d ms", (loadTime / 1000));
}

bool nodeGetValidConfig(void)
{
    return FLAG_VALID_CONFIG;
}
