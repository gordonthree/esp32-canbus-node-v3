/* ============================================================================
 *  ROUTING: NVS LOAD/SAVE (ESP32 ONLY)
 * ========================================================================== */

#ifdef ESP32
#include "can_router.h"
#include <Preferences.h>

extern SemaphoreHandle_t flashMutex;

/* ============================================================================
 *  ESP32 NVS IMPLEMENTATION
 * ========================================================================== */

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

#endif
