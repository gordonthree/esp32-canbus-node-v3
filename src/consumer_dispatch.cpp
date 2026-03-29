#include <Arduino.h>
#include "consumer_handler.h"
#include "consumer_lookup.h"
#include "canbus_msg.h"
#include "can_router.h"
#include "esp_log.h"
static const char *TAG = "consumer_dispatch";

const ConsumerHandlerEntry consumerHandlerTable[] =
{
    /* 0x110–0x1FF: Output commands */
    { 0x110, 0x1FF, handleOutputCommands },

    /* 0x200–0x2FF: Display/UI commands */
    { 0x200, 0x2FF, handleDisplayCommands },

    /* 0x320–0x3FF: Producer configuration commands */
    { 0x320, 0x3FF, handleProducerConfig },

    /* 0x428–0x433: Network submodule configuration commands */
    { 0x428, 0x433, handleNetworkConfig },

    /* 0x434–0x437: NVS commands */
    { CFG_ERASE_NVS_ID, CFG_READ_NVS_ID, handleNvsConfig },


    /* 0x400–0x4FF: Identity/config/network/intro/epoch */
    { 0x400, 0x4FF, handleIdentityConfig },
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

    /* Explicitly ignore 0x300–0x31F (router handles these before consumer) */
    if (id >= 0x300 && id <= 0x31F) {
        ESP_LOGW(TAG, "Received 0x300–0x31F (router handles these before consumer)");
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
    ESP_LOGW(TAG, "Unknown message received 0x%x\n", id);
}
