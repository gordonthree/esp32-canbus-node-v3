#include "can_router.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 *  GLOBAL STORAGE
 * ========================================================================== */

route_entry_t   g_routes[MAX_ROUTES];
producer_cfg_t  g_producerCfg[MAX_SUB_MODULES];

/* Temporary buffer for multi-frame route reception */
static route_entry_t rxRouteBuffer;
static uint8_t       rxRouteIndex = 0;
static uint8_t       rxRouteSlot  = 0xFF;

/* ============================================================================
 *  ROUTING: MULTI-FRAME RECEIVER
 * ========================================================================== */

void handleRouteBegin(const twai_message_t &msg)
{
    if (msg.data_length_code < CFG_ROUTE_BEGIN_DLC)
        return;

    uint8_t route_idx = msg.data[4];
    if (route_idx >= MAX_ROUTES)
        return;

    rxRouteSlot  = route_idx;
    rxRouteIndex = 0;
    memset(&rxRouteBuffer, 0, sizeof(rxRouteBuffer));

    printf("RouteBegin: slot %u\n", route_idx);
}

void handleRouteData(const twai_message_t &msg)
{
    if (msg.data_length_code < CFG_ROUTE_DATA_DLC)
        return;
    if (rxRouteSlot >= MAX_ROUTES)
        return;

    /* Copy 4 bytes (D4..D7) into the struct buffer */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t dst = rxRouteIndex++;
        if (dst >= sizeof(route_entry_t))
            break;
        ((uint8_t*)&rxRouteBuffer)[dst] = msg.data[4 + i];
    }

    printf("RouteData: slot %u idx %u\n", rxRouteSlot, rxRouteIndex);
}

void handleRouteEnd(const twai_message_t &msg)
{
    (void)msg;

    if (rxRouteSlot >= MAX_ROUTES)
        return;

    memcpy(&g_routes[rxRouteSlot], &rxRouteBuffer, sizeof(route_entry_t));
    g_routes[rxRouteSlot].enabled = 1;

    printf("RouteEnd: stored route %u\n", rxRouteSlot);

    rxRouteSlot  = 0xFF;
    rxRouteIndex = 0;
}

void handleRouteDelete(const twai_message_t &msg)
{
    if (msg.data_length_code < CFG_ROUTE_DELETE_DLC)
        return;

    uint8_t idx = msg.data[4];
    if (idx >= MAX_ROUTES)
        return;

    memset(&g_routes[idx], 0, sizeof(route_entry_t));
    printf("RouteDelete: idx %u\n", idx);
}

void handleRoutePurge(const twai_message_t &msg)
{
    (void)msg;
    memset(g_routes, 0, sizeof(g_routes));
    printf("RoutePurge: all cleared\n");
}

/* ============================================================================
 *  ROUTING: NVS LOAD/SAVE (ESP32 ONLY)
 * ========================================================================== */

#ifdef ESP_PLATFORM
#include <Preferences.h>
extern SemaphoreHandle_t flashMutex;

void loadRouteTableFromNVS(void)
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin("route_table", true)) {
        if (prefs.isKey("routes"))
            prefs.getBytes("routes", g_routes, sizeof(g_routes));
        prefs.end();
    }
    xSemaphoreGive(flashMutex);
}

void saveRouteTableToNVS(void)
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin("route_table", false)) {
        prefs.putBytes("routes", g_routes, sizeof(g_routes));
        prefs.end();
    }
    xSemaphoreGive(flashMutex);
}

#else
/* STM32 or other platforms: provide empty stubs */
void loadRouteTableFromNVS(void)  {}
void saveRouteTableToNVS(void)    {}
#endif

void handleRouteWriteNVS(void)
{
    saveRouteTableToNVS();
    printf("RouteTable: saved\n");
}

void handleRouteReadNVS(void)
{
    loadRouteTableFromNVS();
    printf("RouteTable: loaded\n");
}

/* ============================================================================
 *  PRODUCER CONFIG HANDLERS
 * ========================================================================== */

void handleProducerCfg(const twai_message_t &msg)
{
    if (msg.data_length_code < CFG_PRODUCER_CFG_DLC)
        return;

    uint8_t idx = msg.data[4];
    if (idx >= MAX_SUB_MODULES)
        return;

    g_producerCfg[idx].sub_idx  = idx;
    g_producerCfg[idx].rate_hz  = msg.data[5];
    g_producerCfg[idx].flags    = msg.data[6];
    g_producerCfg[idx].reserved = msg.data[7];

    printf("ProducerCfg: sub %u rate %u flags 0x%02X\n",
           idx,
           g_producerCfg[idx].rate_hz,
           g_producerCfg[idx].flags);
}

void handleProducerPurge(const twai_message_t &msg)
{
    (void)msg;
    memset(g_producerCfg, 0, sizeof(g_producerCfg));
    printf("ProducerCfg: purged\n");
}

void handleProducerDefaults(const twai_message_t &msg)
{
    (void)msg;
    memset(g_producerCfg, 0, sizeof(g_producerCfg));
    printf("ProducerCfg: defaults applied\n");
}

void handleProducerApply(void)
{
    /* No-op: config is live immediately */
    printf("ProducerCfg: apply\n");
}

void handleReqProducerCfg(const twai_message_t &msg)
{
    (void)msg;
    /* The caller (ESP32) will send RESP_PRODUCER_CFG_ID frames.
       This module is platform-agnostic, so we do nothing here. */
    printf("ProducerCfg: request received (platform must respond)\n");
}

/* ============================================================================
 *  PRODUCER CONFIG: NVS LOAD/SAVE (ESP32 ONLY)
 * ========================================================================== */

#ifdef ESP_PLATFORM

void loadProducerCfgFromNVS(void)
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin("producer_cfg", true)) {
        if (prefs.isKey("cfg"))
            prefs.getBytes("cfg", g_producerCfg, sizeof(g_producerCfg));
        prefs.end();
    }
    xSemaphoreGive(flashMutex);
}

void saveProducerCfgToNVS(void)
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin("producer_cfg", false)) {
        prefs.putBytes("cfg", g_producerCfg, sizeof(g_producerCfg));
        prefs.end();
    }
    xSemaphoreGive(flashMutex);
}

#else
void loadProducerCfgFromNVS(void) {}
void saveProducerCfgToNVS(void) {}
#endif

void handleProducerWriteNVS(void)
{
    saveProducerCfgToNVS();
    printf("ProducerCfg: saved\n");
}

/* ============================================================================
 *  ROUTE EXECUTION HOOK
 * ========================================================================== */

void checkRoutes(const twai_message_t &msg)
{
    for (uint8_t i = 0; i < MAX_ROUTES; i++) {
        if (!g_routes[i].enabled)
            continue;

        if (msg.identifier != g_routes[i].source_msg_id)
            continue;

        /* TODO: event_type evaluation */
        /* TODO: action execution */

        printf("RouteHit: idx %u src 0x%03X\n", i, msg.identifier);
    }
}
