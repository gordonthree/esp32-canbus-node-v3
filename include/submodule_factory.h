#pragma once

#include <Arduino.h>
#include "node_state.h"
#include "storage.h"
#include "submodule_types.h"
#include "personality_table.h"

#ifdef __cplusplus
extern "C" {
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
int addSubmodule(
    const uint8_t personalityId, 
    const uint8_t* configBytes, 
    size_t configLength);

/** 
 * @brief Clear the configuration of a sub-module in the run-time node configuration table.
 * 
 * @param index The index of the sub-module to clear.
 * 
 * @return True if the sub-module was cleared successfully, false otherwise.
 */
bool clearSubmodule(const uint8_t index);

#ifdef __cplusplus
}
#endif
