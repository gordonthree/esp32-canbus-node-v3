#include "can_router.h"
#include "canbus_msg.h"
#include "consumer_handler.h"
#include "consumer_lookup.h"
#include "esp_log.h"
#include <Arduino.h>

static const char *TAG = "consumer_dispatch";

const ConsumerHandlerEntry consumerHandlerTable[] = {
    /* 0x110–0x13F: Output commands */
    {SW_RESERVED_110_ID, SW_RESERVED_13F_ID, handleOutputCommands},

    /* 0x200–0x22F: Display/UI commands */
    {SET_DISPLAY_OFF_ID, SET_BUTTON_RESERVED_22F_ID, handleDisplayCommands},

    /* 0x320–0x332: Producer configuration commands */
    {PRODUCER_RESERVED_320_ID, PRODUCER_RESERVED_332_ID, handleProducerConfig},

    /* 0x430–0x439: Network submodule configuration commands */
    {CFG_NET_RESERVED_430_ID, CFG_NET_RESERVED_439_ID, handleNetworkConfig},

    /* 0x440–0x449: NVS and system commands */
    {CFG_NVS_RESERVED_440_ID, CFG_NVS_RESERVED_449_ID, handleNvsConfig},

    /* 0x400–0x47F: Identity/config/network/intro/epoch */
    {ACK_INTRO_ID, CFG_RESERVED_47F_ID, handleIdentityConfig},
};

const uint8_t consumerHandlerTableCount =
    (uint8_t)(sizeof(consumerHandlerTable) / sizeof(consumerHandlerTable[0]));

static void send_route_table(void) {
  const int active_route_count = router_get_active_route_count();

  if (active_route_count == 0) {
    ESP_LOGD(TAG, "No active routes to transmit");
    return;
  }

  const uint8_t msgs_per_route = router_get_msgs_per_route();
  const uint16_t total_msg_count = router_get_total_msgs_required();
  const uint32_t node_id = nodeGetNodeID();

  ESP_LOGD(TAG,
           "There are %u active routes using %u messages per route. That is %u "
           "total messages.",
           active_route_count, msgs_per_route, total_msg_count);

  //   can_msg_t *buffer = (can_msg_t *)calloc(total_msg_count *
  //   sizeof(can_msg_t));

  /* Allocate memory for the CAN message buffer */
  can_msg_t *buffer = (can_msg_t *)calloc(total_msg_count, sizeof(*buffer));

  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate %u CAN messages", total_msg_count);
    return;
  }

  int written = router_serialize_all_routes(node_id, buffer, total_msg_count);
  if (written < 0) {
    ESP_LOGE(TAG, "Router serialization failed — buffer too small");
    free(buffer);
    return;
  } else if (written > total_msg_count) {
    ESP_LOGE(
        TAG,
        "Router serialization returned more messages than expected (%d > %d)",
        written, total_msg_count);
    free(buffer);
    return;
  } else {
    ESP_LOGD(TAG, "Serialized %u messages", written);
  }

  // implement function to send messages slowly, to not overwhelm the bus
  for (int i = 0; i < written; i++) {
    canEnqueueMessage(buffer[i].identifier, buffer[i].data,
                      buffer[i].data_length_code);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  free(buffer);
}

void consumeMsg(can_msg_t *msg) {
  const uint16_t id = msg->identifier;

  /* Special case: ROUTE_TAKE_NO_ACTION (0xFFFF, outside normal ranges) */
  if (id == ROUTE_TAKE_NO_ACTION) {
    ESP_LOGV(TAG, "Received ROUTE_TAKE_NO_ACTION");
    /* no action */
    return;
  }

  if (id == DATA_ROUTE_ACK_ID) {
    ESP_LOGV(TAG, "Received DATA_ROUTE_ACK_ID");
    handleNvsConfig(msg); /* handle through the NVS and System config module */
    return;
  }

  if (id == REQ_ROUTE_LIST_ID) {
    ESP_LOGV(TAG, "Received REQ_ROUTE_LIST_ID");
    send_route_table();
    return;
  }

  /* Explicitly ignore 0x300–0x31F (router handles these before consumer) */
  if (id >= 0x300 && id <= 0x31F) {
    ESP_LOGV(TAG, "Received router command 0x%03X after router", id);
    return;
  }

  /* Range-based dispatch */
  for (uint8_t i = 0; i < consumerHandlerTableCount; i++) {
    const ConsumerHandlerEntry *entry = &consumerHandlerTable[i];
    ESP_LOGV(TAG, "Searching for route for 0x%x", id);
    if (id >= entry->startId && id <= entry->endId) {
      ESP_LOGD(TAG, "Found handler for 0x%x at index %d", id, i);
      entry->handler(msg);
      return;
    }
  }

  /* No handler claimed this ID */
  ESP_LOGW(TAG, "Unknown message received 0x%x", id);
}
