#include <Arduino.h>
#include "consumer_handler.h"
#include "consumer_lookup.h"
#include "canbus_msg.h"
#include "can_router.h"
#include "esp_log.h"
static const char *TAG = "consumer_dispatch";

const ConsumerHandlerEntry consumerHandlerTable[] =
{
    /* 0x110–0x13F: Output commands */
    { SW_RESERVED_110_ID, SW_RESERVED_13F_ID, handleOutputCommands },

    /* 0x200–0x22F: Display/UI commands */
    { SET_DISPLAY_OFF_ID, SET_BUTTON_RESERVED_22F_ID, handleDisplayCommands },

    /* 0x320–0x332: Producer configuration commands */
    { PRODUCER_RESERVED_320_ID, PRODUCER_RESERVED_332_ID, handleProducerConfig },

    /* 0x430–0x439: Network submodule configuration commands */
    { CFG_NET_RESERVED_430_ID, CFG_NET_RESERVED_439_ID, handleNetworkConfig },

    /* 0x440–0x449: NVS and system commands */
    { CFG_NVS_RESERVED_440_ID, CFG_NVS_RESERVED_449_ID, handleNvsConfig },

    /* 0x400–0x47F: Identity/config/network/intro/epoch */
    { ACK_INTRO_ID, CFG_RESERVED_47F_ID, handleIdentityConfig },
};

const uint8_t consumerHandlerTableCount =
    (uint8_t)(sizeof(consumerHandlerTable) / sizeof(consumerHandlerTable[0]));

void consumeMsg(can_msg_t *msg)
{
    const uint16_t id = msg->identifier;

    /* Special case: ROUTE_TAKE_NO_ACTION (0xFFFF, outside normal ranges) */
    if (id == ROUTE_TAKE_NO_ACTION) {
        ESP_LOGD(TAG, "Received ROUTE_TAKE_NO_ACTION");
        /* no action */
        return;
    }

    if (id == DATA_ROUTE_ACK_ID) {
        ESP_LOGD(TAG, "Received DATA_ROUTE_ACK_ID");
        handleNvsConfig(msg); /* handle through the NVS and System config module */
        return;
    }

    /* Explicitly ignore 0x300–0x31F (router handles these before consumer) */
    if (id >= 0x300 && id <= 0x31F) {
        ESP_LOGW(TAG, "Received router command 0x%03X (before consumer)", id);
        return;
    }

    /* Range-based dispatch */
    for (uint8_t i = 0; i < consumerHandlerTableCount; i++) {
        const ConsumerHandlerEntry *entry = &consumerHandlerTable[i];
        ESP_LOGD(TAG, "Searching for route for 0x%x", id);
        if (id >= entry->startId && id <= entry->endId) {
            ESP_LOGI(TAG, "Found handler for 0x%x at index %d", id, i);
            entry->handler(msg);
            return;
        }
    }

    /* No handler claimed this ID */
    ESP_LOGW(TAG, "Unknown message received 0x%x", id);
}
