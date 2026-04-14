#pragma once

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

#include "can_router.h"        // for g_routes + MAX_ROUTES
// #include "can_producer.h"      // for producer  (your producer config struct)
#include "node_state.h"        // for shared objects from main.cpp
#include "submodule_types.h"   /**< Sub-module type definitions */
#include "crc16.h"             /**< CRC16 functions */

#define ROUTER_VERSION        (6U)
#define PRODUCER_VERSION      (2U)
#define NODEINFO_VERSION      (7U)

#ifndef IGNORE_CRC_ERROR
#define IGNORE_CRC_ERROR false
#endif

/**
 * @enum ConfigStatus
 * @brief Result codes for NVS operations.
 */
enum ConfigStatus {
    CFG_OK = 0,        /**< Load successful and CRC valid */
    CFG_VERS_ERROR,    /**< Config version mismatch */
    CFG_ERR_MUTEX,     /**< Failed to acquire flash mutex */
    CFG_ERR_NOT_FOUND, /**< No configuration exists in NVS */
    CFG_ERR_CRC,       /**< Data found but CRC is invalid (corrupt) */
    CFG_ERR_CRC_MISS,  /**< CRC key not found in NVS */
    CFG_ERR_NVS_OPEN   /**< Failed to open NVS namespace */
};

/* NVS namespace definitions */
static constexpr const char* ROUTE_NS        = "route_table";
static constexpr const char* ROUTE_KEY       = "route_data";
static constexpr const char* ROUTE_CRC_KEY   = "route_crc";
static constexpr const char* ROUTE_VERSION   = "route_version";

static constexpr const char* PROD_NS         = "producer_table";
static constexpr const char* PROD_KEY        = "producer_data";
static constexpr const char* PROD_CRC_KEY    = "producer_crc";
static constexpr const char* PROD_VERSION    = "producer_version";

static constexpr const char* NODE_NS         = "node_cfg";
static constexpr const char* NODE_CRC_KEY    = "node_crc";
static constexpr const char* NODE_DATA_KEY   = "node_data";
static constexpr const char* NODE_VERSION    = "node_version";

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

/** Erase the node configuration and CRC from NVS */
ConfigStatus eraseConfigNvs();
/** Save the node configuration and CRC to NVS */
ConfigStatus saveConfigNvs();
/** Load the node configuration and CRC from NVS */
ConfigStatus loadConfigNvs();

/** Check if the node configuration is valid */
bool nodeGetValidConfig(void);


/* =========================================================================== 
 * HELPER FUNCTIONS 
 * ========================================================================= */

nodeInfo_t makeSanitizedNodeInfo(const nodeInfo_t *src);