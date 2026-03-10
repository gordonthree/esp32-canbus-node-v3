#include "storage.h"

// Global mutex for NVS access (declared in main.cpp)
extern SemaphoreHandle_t flashMutex;

// NVS namespaces
static constexpr const char* ROUTE_NS     = "route_table";
static constexpr const char* ROUTE_KEY    = "routes";

static constexpr const char* PROD_NS      = "producer_cfg";
static constexpr const char* PROD_KEY     = "cfg";

/* ============================================================================
 *  ROUTE TABLE LOAD/SAVE
 * ========================================================================== */

void loadRoutesFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, true)) {
        if (prefs.isKey(ROUTE_KEY))
            prefs.getBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveRoutesToNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(ROUTE_NS, false)) {
        prefs.putBytes(ROUTE_KEY, g_routes, sizeof(g_routes));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

/* ============================================================================
 *  PRODUCER CONFIG LOAD/SAVE
 * ========================================================================== */

void loadProducerCfgFromNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(PROD_NS, true)) {
        if (prefs.isKey(PROD_KEY))
            prefs.getBytes(PROD_KEY, g_producerCfg, sizeof(g_producerCfg));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}

void saveProducerCfgToNVS()
{
    if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return;

    Preferences prefs;
    if (prefs.begin(PROD_NS, false)) {
        prefs.putBytes(PROD_KEY, g_producerCfg, sizeof(g_producerCfg));
        prefs.end();
    }

    xSemaphoreGive(flashMutex);
}
