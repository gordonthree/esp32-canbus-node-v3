#ifndef __CAN_ROUTER_H__
#define __CAN_ROUTER_H__

#include "canbus_project.h"


#include <stdint.h>
#include "driver/twai.h"

#define MAX_ROUTES        (8U)
#define PRODUCER_FLAG_ENABLED      (0x01U)
#define PRODUCER_FLAG_CHANGE_ONLY  (0x02U)

typedef struct __attribute__((packed)) {
    uint8_t  parameters[8];      // configuration parameters (8 bytes)
    uint8_t  source_node_id[4];  // producer node ID (4 bytes)

    uint16_t source_msg_id;      // source CAN message ID (2 bytes)
    uint16_t target_msg_id;      // target CAN message ID (2 bytes)
    uint8_t  source_msg_dlc;     // source DLC (1 byte)
    uint8_t  target_msg_dlc;     // target DLC (1 byte)

    uint8_t  source_sub_idx;     // source submodule index (1 byte)
    uint8_t  target_sub_idx;     // target submodule index (1 byte)

    uint8_t  event_type;         // event type (1 byte)
    uint8_t  action_type;        // action type (1 byte)

    uint8_t  param_len;          // number of parameter bytes used (1 byte)

    uint8_t  enabled;            // enabled flag (1 byte)
} route_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t sub_idx;   // submodule index
    uint8_t rate_hz;   // broadcast rate (0 = disabled)
    uint8_t flags;     // PRODUCER_FLAG_*
    uint8_t reserved;  // future use
} producer_cfg_t;

extern route_entry_t   g_routes[MAX_ROUTES];
extern producer_cfg_t  g_producerCfg[MAX_SUB_MODULES];

void checkRoutes(const twai_message_t &msg);
void handleRouteBegin(const twai_message_t &msg);
void handleRouteData(const twai_message_t &msg);
void handleRouteEnd(const twai_message_t &msg);
void handleRouteDelete(const twai_message_t &msg);
void handleRoutePurge(const twai_message_t &msg);
void handleRouteWriteNVS(void);
void handleRouteReadNVS(void);

void handleProducerCfg(const twai_message_t &msg);
void handleProducerWriteNVS(void);
void handleProducerPurge(const twai_message_t &msg);
void handleProducerDefaults(const twai_message_t &msg);
void handleProducerApply(void);
void handleReqProducerCfg(const twai_message_t &msg);

void loadRouteTableFromNVS(void);
void saveRouteTableToNVS(void);
void loadProducerCfgFromNVS(void);
void saveProducerCfgToNVS(void);

#endif // __CAN_ROUTER_H__

