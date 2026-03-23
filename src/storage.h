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
static constexpr const char* ROUTE_KEY       = "route_data";
static constexpr const char* ROUTE_CRC_KEY   = "route_crc";


static constexpr const char* PROD_NS         = "producer_table";
static constexpr const char* PROD_KEY        = "producer_data";
static constexpr const char* PROD_CRC_KEY    = "producer_crc";


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
 *  SUBMODULE MANAGEMENT API
 * ========================================================================== */

/** 
 * @brief Add a new sub-module to the node configuration.
 * 
 * @param personalityId The index of the personality template.
 * @param config The configuration data for the sub-module.
 * 
 * @return The index of the new sub-module in the node configuration, -1 indicates failure.
 */
int addSubmodule(const uint8_t personalityId, const uint8_t* configBytes, size_t configLength);

/** 
 * @brief Remove a sub-module from the node configuration.
 * 
 * @param index The index of the sub-module to remove.
 * 
 * @return True if the sub-module was removed successfully, false otherwise.
 */
bool removeSubmodule(const uint8_t index);


/* ============================================================================
 *  NODE CONFIG STORAGE API
 * ========================================================================== */
void loadNodeDefaults();
void handleReadCfgNVS();
void handleEraseCfgNVS();
/** Erase the node configuration and CRC from NVS */
ConfigStatus eraseConfigNvs();
/** Save the node configuration and CRC to NVS */
ConfigStatus saveConfigNvs(const nodeInfo_t& node);
/** Load the node configuration and CRC from NVS */
ConfigStatus loadConfigNvs(nodeInfo_t& node);
/** Save a sub-module and CRC to NVS */
ConfigStatus saveSubModuleNvs(const subModule_t& subModule, uint8_t index);



/* =========================================================================== 
 * HELPER FUNCTIONS 
 * ========================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16_ccitt(const uint8_t *data, 
                     uint16_t length);

uint16_t crc16_ccitt_update(uint16_t crc, 
                            const uint8_t *data, 
                            uint16_t length);

#ifdef __cplusplus
}
#endif