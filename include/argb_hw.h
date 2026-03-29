#pragma once

/*
 * =========================================================================
 * Includes
 * =========================================================================
 */

#include <Arduino.h>

#include "canbus_project.h" /**< my various CAN functions and structs */
#include "can_router.h"     /**< CAN routing (subcriber) routines and constants */
#include "node_state.h"     /**< Node and sub-module state table */
#include "colorpalette.h"   /**< my colors */

#include "submodule_types.h"

/*
 * =========================================================================
 * Constants and macros
 * =========================================================================
 */

/*
 * =========================================================================
 * Public functions
 * =========================================================================
 */

#ifdef __cplusplus
extern "C"
{
#endif

    void initArgbHardware(uint8_t index, subModule_t *sub);
    void argbSetStripColor(uint8_t index, uint8_t colorIndex);

#ifdef __cplusplus
}
#endif
