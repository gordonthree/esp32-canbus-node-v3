#pragma once

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

#include "can_router.h"        // for g_routes + MAX_ROUTES
// #include "can_producer.h"      // for producer  (your producer config struct)
#include "node_state.h"        // for shared objects from main.cpp
#include "submodule_types.h"   /**< Sub-module type definitions */


/**
 * @enum ConfigStatus
 * @brief Result codes for NVS operations.
 */
enum ConfigStatus {
    CFG_OK = 0,        /**< Load successful and CRC valid */
    CFG_ERR_MUTEX,     /**< Failed to acquire flash mutex */
    CFG_ERR_NOT_FOUND, /**< No configuration exists in NVS */
    CFG_ERR_CRC,       /**< Data found but CRC is invalid (corrupt) */
    CFG_ERR_CRC_MISS,  /**< CRC key not found in NVS */
    CFG_ERR_NVS_OPEN   /**< Failed to open NVS namespace */
};

// NVS namespaces
static constexpr const char* ROUTE_NS        = "route_table";
static constexpr const char* ROUTE_KEY       = "routes";

static constexpr const char* PROD_NS         = "producer_cfg";
static constexpr const char* PROD_KEY        = "cfg";

static constexpr const char* NODE_NS         = "node_cfg";
static constexpr const char* NODE_CRC_KEY    = "node_crc";
static constexpr const char* NODE_DATA_KEY   = "node_data";

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  ROUTE TABLE STORAGE API
 * ========================================================================== */

/**
 * @brief Load the route table from NVS into g_routes[].
 *        Safe to call at startup or when the router requests a reload.
 */
void loadRoutesFromNVS(void);

/**
 * @brief Save the current route table (g_routes[]) to NVS.
 *        Called when the router sets the "save requested" flag.
 */
void saveRoutesToNVS(void);


/* ============================================================================
 *  PRODUCER CONFIG STORAGE API
 * ========================================================================== */

/**
 * @brief Load producer configuration from NVS into g_producerCfg.
 */
void loadProducerCfgFromNVS(void);

/**
 * @brief Save producer configuration (g_producerCfg) to NVS.
 */
void saveProducerCfgToNVS(void);


#ifdef __cplusplus
}
#endif


/* ============================================================================
 *  NODE CONFIG STORAGE API
 * ========================================================================== */
void loadNodeDefaults();
void handleReadCfgNVS();
void handleEraseCfgNVS();
ConfigStatus eraseConfigNvs();
ConfigStatus saveConfigNvs(const nodeInfo_t& node);
ConfigStatus loadConfigNvs(nodeInfo_t& node);