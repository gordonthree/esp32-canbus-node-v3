#pragma once

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

#include "can_router.h"        // for g_routes + MAX_ROUTES
#include "can_producer.h"      // for g_producerCfg (your producer config struct)

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
