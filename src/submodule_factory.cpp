#include "submodule_factory.h"
#include "string.h"
#include "esp_log.h"

static const char *TAG = "submod_factory";

static void factoryAddInternal(subModule_t *sub);

static void factoryAddNetwork(subModule_t *sub,
                             const uint8_t *configBytes,
                             size_t configLength);

static int factoryFindFreeSubmoduleSlot(void);
static int factoryValidatePersonality(uint8_t personalityId);
/* ============================================================================
 *  PRIVATE FUNCTIONS
 * ========================================================================== */

/**
 * @brief Validate personalityId 
 *
 * @param personalityId  Template personality ID to validate.
 * @return int  Index of a free submodule slot, or -1 on error.
 */
static int factoryValidatePersonality(uint8_t personalityId)
{
    nodeInfo_t *node = nodeGetInfo();

    /* Validate personality ID */
    if (personalityId >= PERSONALITY_MAX)
    {
        ESP_LOGW(TAG, "[FACTORY] Invalid personality ID: %u", personalityId);
        return -1;
    }

    /* Validate submodule count */
    if (node->subModCnt >= MAX_SUB_MODULES)
    {
        ESP_LOGE(TAG,
                 "[FACTORY] CRITICAL ERROR: subModCnt=%u exceeds max=%u",
                 node->subModCnt, MAX_SUB_MODULES);
        return -1;
    }

    /* Validation passed, search for a free sub-module slot */
    const int freeSlot = factoryFindFreeSubmoduleSlot();
    if (freeSlot >= 0)
    {
        ESP_LOGD(TAG, "[FACTORY] Found free sub-module slot: %d", freeSlot);
        return freeSlot;
    }

    ESP_LOGW(TAG, "[FACTORY] Submodule table is full.");
    return -1;
}

/** Add internal-type specific fields to the sub-module */
 static void factoryAddInternal(subModule_t *sub)
{

    /* Get pointer to the personality definition for this sub-module */
    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    /* Copy introMsgId and introMsgDLC from the personality into the sub-module */
    sub->introMsgId = p->introMsgId;
    sub->introMsgDLC = p->introMsgDlc;

    /* Get default producer period_ms from personality definition */
    const uint16_t period_ms = p->period_ms;

    /* validate period_ms */
    if (period_ms == PRODUCER_PERIOD_DISABLED)
    {
        /* 0 == do not publish, assign safe default rate */
        sub->producer_period_ms = PRODUCER_PERIOD_1000MS; 
        /* active but not enabled */ 
        sub->producer_flags = PRODUCER_FLAG_ACTIVE; 
    }
    else if (period_ms >= PRODUCER_PERIOD_10MS) /* period >= 10ms */
    {
        sub->producer_period_ms = period_ms;
        sub->producer_flags = PRODUCER_FLAG_ACTIVE |
                              PRODUCER_FLAG_PUBLISH_ENABLED;
    }

}

static void factoryAddNetwork(subModule_t *sub,
                             const uint8_t *configBytes,
                             size_t configLength)
{

    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    /* Copy rawConfig (3 bytes max) */
    size_t copyLen = (configLength > 3) ? 3 : configLength;
    memcpy(sub->config.rawConfig, configBytes, copyLen);

    /* Initialize network fields if CAP_NETWORK is set */
    if (p->capabilities & CAP_NETWORK)
    {
        /* Expecting configBytes to contain a 32-bit nodeID */
        if (configLength >= 4)
        {
            sub->networkNodeId =
                ((uint32_t)configBytes[0] << 24) |
                ((uint32_t)configBytes[1] << 16) |
                ((uint32_t)configBytes[2] << 8) |
                ((uint32_t)configBytes[3]);
        }

        /* zero timestamp */
        sub->lastSeen = 0;                                 
        /* clear network config */
        memset(sub->netConfig, 0, sizeof(sub->netConfig)); 
    }
}

/**
 * @brief Locate the first free submodule slot.
 *
 * @return int  Index of a free slot, or -1 if none available.
 */
static int factoryFindFreeSubmoduleSlot(void)
{
    nodeInfo_t *node = nodeGetInfo();

    for (int i = 0; i < MAX_SUB_MODULES; i++)
    {
        if (node->subModule[i].personalityIndex == 0xFF)
        {
            return i;   /* success */
        }
    }

    return -1;  /* no free slot */
}

/* ============================================================================
 *  SUBMODULE MANAGEMENT PUBLIC API
 * ========================================================================== */

/**
 *
 * @brief Add a new sub-module to the node configuration. This function will call helper
 * functions to add specific sub-module types to the node runtime configuration.
 *
 * Creates entries in node.subModule array, as well as the personality table array.
 *
 * @param personalityId The index of the personality template.
 * @param config The configuration data for the sub-module.
 * @param configLength The length of the configuration data array.
 *
 * @return The index of the new sub-module in the node configuration, -1 indicates failure.
 */
int addSubmodule(const uint8_t personalityId,
                 const uint8_t *configBytes,
                 size_t configLength)
{

    
    /* === VALIDATION CHECKS === */
    const int result = factoryValidatePersonality(personalityId);
    
    if (result < 0) /* validation failed */
        return result;

    /* success, convert to uint8_t */
    const uint8_t index = (uint8_t)result;

    /* === INITIALIZATION === */

    /* Initialize the new submodule */
    subModule_t *sub = nodeGetSubModule(index);
    memset(sub, 0, sizeof(subModule_t));

    /* retrieve a copy of the personality template */
    personalityDef_t pTemplate = getPersonalityTemplate(personalityId);

    /* Find a free slot in the runtime personality table */
    const int personalityResult = getFreePersonalitySlot();
    if (personalityResult < 0)
    {
        ESP_LOGE(TAG, "[FACTORY] Failed to find free personality slot.");
        return -1;
    }
    const uint8_t freeSlot = (uint8_t)personalityResult;

    /* Access personality table directly, for read-write access */
    personalityDef_t *p = &runtimePersonalityTable[freeSlot];

    /* Copy template onto runtime table */
    *p = pTemplate;

    /* Set the personality index equal to the template id */
    sub->personalityIndex = freeSlot;

    /*
     * Route to the correct helper function based on type
     * eg network or internal submodule */

    const uint8_t builderFlags = p->flags;
    if (builderFlags & BUILDER_FLAG_IS_INTERNAL)
    {
        factoryAddInternal(sub);
    }
    else if (builderFlags & BUILDER_FLAG_IS_NETWORK_NODE)
    {
        factoryAddNetwork(sub, configBytes, configLength);
    }

    /* Mark runtime flags for saving to NVS */
    sub->submod_flags = SUBMOD_FLAG_DIRTY;

    /* Increment count */
    nodeIncSubModuleCount();

    return index;
}

bool clearSubmodule(const uint8_t index)
{
    if (index >= MAX_SUB_MODULES)
    {
        ESP_LOGW(TAG, "[NVS] Submodule index %d out of range", index);
        return false;
    }

    subModule_t *sub = nodeGetSubModule(index);

    /* If the slot is already empty, nothing to do */
    if (sub->personalityIndex == 0xFF)
    {
        ESP_LOGW(TAG, "[NVS] Submodule index %d already clear", index);
        return false;
    }

    /* Clear the last entry, clear the memory not the pointer */
    memset(sub, 0xFF, sizeof(subModule_t));

    /* Persist to NVS, eventually */
    sub->submod_flags |= SUBMOD_FLAG_DIRTY; 

    ESP_LOGD(TAG, "[NVS] Submodule index %d cleared", index);
    return true;
}