#ifndef __CAN_ROUTER_H__
#define __CAN_ROUTER_H__


#include "canbus_project.h"
#include <stdint.h>
#include "driver/twai.h"

/* ============================================================================
 *  CONSTANTS
 * ========================================================================== */

#define MAX_ROUTES                 (8U)
#define PRODUCER_FLAG_ENABLED      (0x01U)
#define PRODUCER_FLAG_CHANGE_ONLY  (0x02U)
#define PRODUCER_FLAG_RESERVED1    (0x04U)
#define PRODUCER_FLAG_RESERVED2    (0x08U)

/* ============================================================================
 *  DATA STRUCTURES
 * ========================================================================== */

typedef struct __attribute__((packed)) {
    uint8_t  parameters[8];      /* configuration parameters (8 bytes) */
    uint8_t  source_node_id[4];  /* producer node ID (4 bytes) */

    uint16_t source_msg_id;      /* source CAN message ID */
    uint16_t target_msg_id;      /* target CAN message ID */
    uint8_t  source_msg_dlc;     /* source DLC */
    uint8_t  target_msg_dlc;     /* target DLC */

    uint8_t  source_sub_idx;     /* source submodule index */
    uint8_t  target_sub_idx;     /* target submodule index */

    uint8_t  event_type;         /* event type */
    uint8_t  action_type;        /* action type */

    uint8_t  param_len;          /* number of parameter bytes used */

    uint8_t  enabled;            /* enabled flag */
} route_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t sub_idx;   /* submodule index */
    uint8_t rate_hz;   /* broadcast rate (0 = disabled) */
    uint8_t flags;     /* PRODUCER_FLAG_* */
    uint8_t reserved;  /* future use */
} producer_cfg_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  GLOBALS
 * ========================================================================== */

extern route_entry_t   g_routes[MAX_ROUTES];
extern producer_cfg_t  g_producerCfg[MAX_SUB_MODULES];

/* ============================================================================
 *  ROUTING API
 * ========================================================================== */

void checkRoutes(const twai_message_t *msg);

void handleRouteBegin(const twai_message_t *msg);
void handleRouteData(const twai_message_t *msg);
void handleRouteEnd(const twai_message_t *msg);
void handleRouteDelete(const twai_message_t *msg);
void handleRoutePurge(const twai_message_t *msg);

void handleRouteWriteNVS(void);
void handleRouteReadNVS(void);

/* ============================================================================
 *  PRODUCER CONFIG API
 * ========================================================================== */

void handleProducerCfg(const twai_message_t *msg);
void handleProducerWriteNVS(void);
void handleProducerPurge(const twai_message_t *msg);
void handleProducerDefaults(const twai_message_t *msg);
void handleProducerApply(void);
void handleReqProducerCfg(const twai_message_t *msg);

/* ============================================================================
 *  NVS LOAD/SAVE API (platform-specific)
 * ========================================================================== */

void loadRouteTableFromNVS(void);
void saveRouteTableToNVS(void);

void loadProducerCfgFromNVS(void);
void saveProducerCfgToNVS(void);

/* ============================================================================
 *  END C LINKAGE
 * ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_ROUTER_H__ */
