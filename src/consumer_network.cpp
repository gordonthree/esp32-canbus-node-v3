#include "consumer_handler.h"
#include "byte_conversion.h"
#include "storage.h" /* NVS storage routines */
#include "esp_log.h"

static const char *TAG = "consumer_network";

static bool pendingNetNodeValid = false;
static bool pendingNetDataValid = false;
static uint32_t pendingRemoteNodeId = 0;
static uint8_t pendingRemoteIndex = 0;
static uint8_t pendingPersonalityId = 0;

static void handleAddNetworkSubmodule(void);

static void handleAddNetworkSubmodule(void)
{
    /* Only proceed when both messages have arrived and validated */
    if (!pendingNetDataValid ||
        !pendingNetNodeValid)
        return;

    uint8_t cfg[NETWORK_NODE_DATA_LEN] = {0}; /* reserve 8 bytes, zero out */
    packUint32ToBytes(pendingRemoteNodeId, cfg);
    cfg[DATA_SUBMOD_IDX_OFFSET] = pendingRemoteIndex;

    int idx = addSubmodule(pendingPersonalityId, cfg, sizeof(cfg));
    if (idx < 0)
    {
        ESP_LOGW(TAG, "[ERR] Failed to add network submodule");
    }
    else
    {
        ESP_LOGI(TAG, "[CFG] Added network submodule: node=0x%08lX index=%u submod=%d)",
                 (unsigned long)pendingRemoteNodeId,
                 pendingRemoteIndex,
                 idx);
    }

    /* Clear pending state */
    pendingNetDataValid = false;
    pendingNetNodeValid = false;
}

void handleNetworkConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4]; /* byte 4 holds the sub module index */

    /* skip local messages for all functions in this block */
    if (blockLocalMsg(msg))
        return;

    ESP_LOGI(TAG, "Processing network command 0x%03X for sub module %d", msg->identifier, modIdx);

    switch (msg->identifier)
    {
    case CFG_NET_NODE_ADD_ID:
    {
        /* Extract remote node ID */
        pendingRemoteNodeId = unpackBytestoUint32(&msg->data[4]);

        /* Reject invalid node ID (0 is reserved for broadcast) */
        if (pendingRemoteNodeId == 0)
        {
            ESP_LOGW(TAG, "[CONSUMER] Error: Invalid remote node ID (0)");
            pendingNetNodeValid = false;
            break;
        }

        pendingNetNodeValid = true;

        ESP_LOGI(TAG, "[CONSUMER] Pending remoteNodeId = 0x%08lX",
                 (unsigned long)pendingRemoteNodeId);

        /* Attempt creation if CFG already arrived */
        handleAddNetworkSubmodule();
        break;
    }

    case CFG_NET_NODE_DATA_ID:
    {
        /* Extract index + personality */
        const uint8_t remoteIndex = msg->data[4];
        const uint8_t personalityId = msg->data[5];

        /* Validate personalityId against template table */
        if (!isValidTemplatePersonality(personalityId))
        {
            ESP_LOGW(TAG, "[CONSUMER] Error: Invalid template personalityId: %u", personalityId);
            pendingNetDataValid = false;
            break;
        }

        /* Store pending data */
        pendingRemoteIndex = remoteIndex;
        pendingPersonalityId = personalityId;
        pendingNetDataValid = true;

        ESP_LOGI(TAG, "[CFG] Pending remoteIndex=%u personalityId=%u",
                 pendingRemoteIndex, pendingPersonalityId);

        /* Attempt creation if ADD already arrived */
        handleAddNetworkSubmodule();
        break;
    }

    case CFG_NET_NODE_DEL_ID:
    {
        /* Extract remote node ID */
        const uint32_t remoteNode = unpackBytestoUint32(&msg->data[4]);

        /* Reject invalid node ID */
        if (remoteNode == 0)
        {
            ESP_LOGW(TAG, "[ERR] DEL request for invalid node ID (0)");
            break;
        }

        /* Search for matching network submodule */
        for (uint8_t i = 0; i < nodeGetInfo()->subModCnt; i++)
        {
            if (nodeGetSubModule(i)->networkNodeId == remoteNode)
            {
                removeSubmodule(i);
                ESP_LOGI(TAG, "[CFG] Removed network submodule for node 0x%08lX",
                         (unsigned long)remoteNode);
                break;
            }
        }

        break;
    }
    } /* end switch */
} /* end handleNetworkConfig() */