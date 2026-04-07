#include "submodule_factory.h"
#include "string.h"
#include "esp_log.h"

static const char *TAG = "submod_factory";

static void factoryAddInternal(subModule_t *sub);

static void factoryAddNetwork(subModule_t *sub,
                              const uint8_t *configBytes,
                              const size_t configLength);

static int factoryFindFreeSubmoduleSlot(void);
static int factoryValidatePersonality(const uint8_t personalityId);
/* ============================================================================
 *  PRIVATE FUNCTIONS
 * ========================================================================== */

/**
 * @brief Validate personalityId
 *
 * @param personalityId  Template personality ID to validate.
 * @return int  Index of a free submodule slot, or -1 on error.
 */
static int factoryValidatePersonality(const uint8_t personalityId)
{
    nodeInfo_t *node = nodeGetInfo();

    /* Validate personality ID is in-range */
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
        ESP_LOGV(TAG, "[FACTORY] Found free sub-module slot: %d", freeSlot);
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

    /* Set default flags, the user can change these during runtime if desired */
    sub->submod_flags |= SUBMOD_FLAG_INTERNAL | SUBMOD_FLAG_READ_ONLY;

    /* Get default producer period_ms from personality definition */
    const uint16_t period_ms = p->period_ms;

    /* validate period_ms */
    if (period_ms == PRODUCER_PERIOD_DISABLED)
    {
        /* publishing disabled, still assign safe default rate */
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
    else /* period < 10ms */
    {
        ESP_LOGW(TAG, "[FACTORY] Invalid producer period %u - using %u as default.", period_ms, PRODUCER_PERIOD_1000MS);
        /* incorrect rate; assign a safe default rate */
        sub->producer_period_ms = PRODUCER_PERIOD_1000MS;
        sub->producer_flags = PRODUCER_FLAG_ACTIVE |
                              PRODUCER_FLAG_PUBLISH_ENABLED;
    }
}

static void factoryAddNetwork(subModule_t *sub,
                              const uint8_t *configBytes,
                              const size_t configLength)
{

    const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);

    /* Set default flags, the user can change these and they will persist in NVS */
    sub->submod_flags |= SUBMOD_FLAG_NETWORK;

    /* Expecting configBytes to contain a 32-bit nodeID */
    if (configLength >= 4)
    {
        sub->networkNodeId =
            ((uint32_t)configBytes[0] << 24) |
            ((uint32_t)configBytes[1] << 16) |
            ((uint32_t)configBytes[2] << 8) |
            ((uint32_t)configBytes[3]);
    }

    if (configLength >= 5)
    {
        sub->netConfig[0] = configBytes[4];
    }

    /* 3 more bytes are available in configBytes, reserved for future use */

    /* zero timestamp - reserved for future use */
    sub->lastSeen = 0;

    /* Mark the sub-module as dirty so main saves it to NVS */
    sub->submod_flags |= SUBMOD_FLAG_DIRTY;
}

/**
 * @brief Locate the first free submodule slot.
 *
 * @return int  Index of a free slot, or -1 if none available.
 */
static int factoryFindFreeSubmoduleSlot(void)
{
    const nodeInfo_t *node = nodeGetInfo();

    for (int i = 0; i < MAX_SUB_MODULES; i++)
    {
        if (node->subModule[i].personalityIndex == 0xFF)
        {
            return i; /* success */
        }
    }

    return -1; /* no free slot */
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

    /* validate personalityId and find a free slot */
    const int result = factoryValidatePersonality(personalityId);

    if (result < 0) /* validation failed */
        return result;

    /* success, convert to uint8_t */
    const uint8_t index = (uint8_t)result;

    /* === INITIALIZATION === */

    /* Get pointer to the new submodule */
    subModule_t *sub = nodeGetSubModule(index);

    /* retrieve a copy of the personality template */
    personalityDef_t pTemplate =
        getPersonalityTemplate(personalityId);

    /* validate template */
    if (pTemplate.personalityId == PERS_NONE)
    { /* template not found */
        ESP_LOGE(TAG, "[FACTORY] Failed to find personality template.");
        return -1;
    }

    /* Access runtime personality table directly for read-write access */
    personalityDef_t *p = &runtimePersonalityTable[index];

    /* Make sure the matching slot is free */
    if (p->personalityId != 0xFF)
    {
        ESP_LOGE(TAG, "[FACTORY] Matching personality slot %u is not free.", index);
        return -1;
    }

    /* initialize new submodule */
    memset(sub, 0, sizeof(subModule_t));

    /* Copy template onto runtime table */
    *p = pTemplate;

    /* Update submodule personality index */
    sub->personalityIndex = index;

    /* Update submodule personality id */
    sub->personalityId = p->personalityId;

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

    ESP_LOGI(TAG, "\n[FACTORY] Submodule added at index %u sub.pId: %u sub.pIdx: %u pers.pId: %u\n",
             index, sub->personalityId, sub->personalityIndex, p->personalityId);

    /* Increment count */
    nodeIncSubModuleCount();

    return index;
}

bool clearSubmodule(const uint8_t index)
{
    if (index >= MAX_SUB_MODULES)
    {
        ESP_LOGW(TAG, "[FACTORY] Submodule index %d out of range", index);
        return false;
    }

    subModule_t *sub = nodeGetSubModule(index);

    /* If the slot is already empty, nothing to do */
    if (sub->personalityIndex == 0xFF)
    {
        ESP_LOGV(TAG, "[FACTORY] Submodule index %d already clear", index);
        return true;
    }

    /* Clear the submodule; clear the memory not the pointer */
    memset(sub, 0xFF, sizeof(subModule_t));

    /* Persist to NVS, eventually, overwrite the flag field */
    sub->submod_flags = SUBMOD_FLAG_DIRTY;

    ESP_LOGD(TAG, "[FACTORY] Submodule index %d cleared", index);
    return true;
}

void clearPersonalitySlot(const uint8_t idx)
{
    if (idx >= MAX_RUNTIME_PERSONALITIES)
    {
        ESP_LOGW(TAG, "[FACTORY] Personality index %d out of range", idx);
        return;
    }

    /* Clear the personality slot */
    memset(&runtimePersonalityTable[idx], 0xFF, sizeof(personalityDef_t));
}