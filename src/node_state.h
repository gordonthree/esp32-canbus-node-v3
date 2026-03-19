#pragma once

#include <Arduino.h>

#include "canbus_project.h"   // for node_t, subModule_t, etc.
#include "submodule_types.h"   /**< Sub-module type definitions */
#include "personality_table.h" /**< Node and sub-module personality table */

/* ============================================================================
 *  GLOBAL VARIABLES
 * ============================================================================ */ 

extern volatile bool FLAG_VALID_CONFIG;

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================ */
/*  NODE STATE API
 * ============================================================================ */

/* Node state accessor functions */
subModule_t* nodeGetSubModule(const uint8_t sub_idx);
runTime_t* nodeGetRuntime(const uint8_t sub_idx);
const uint8_t nodeGetSubModuleCount(void);


/* Flags accessor functions */
uint8_t* nodeGetProducerFlags(const uint8_t sub_idx) ;
uint8_t* nodeGetSubmodFlags(const uint8_t sub_idx);
uint8_t* nodeGetRouterFlags(const uint8_t sub_idx);
void nodeSetProducerFlags(const uint8_t sub_idx, uint8_t flags);
void nodeSetSubmodFlags(const uint8_t sub_idx, uint8_t flags);
void nodeSetRouterFlags(const uint8_t sub_idx, uint8_t flags);


/* ============================================================================
 *  GLOBAL FUNCTIONS
 * ============================================================================ */ 

 /** @brief Send message to the CAN bus, lives in main.cpp */
void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc );

/** pretty print the in-memory node config database */
void printNodeInfo(const nodeInfo_t* node);



/* ============================================================================
 *  NODE STATE API
 * ============================================================================ */ 

/**
 * @struct outputTracker_t
 * @brief Represents the state of an output module.
 *
 * This struct is used to track the state of an output module. It contains the following fields:
 *
 * @param nextActionTime: The timestamp for the next state change.
 * @param currentStep: The current step in a multi-stage pattern (strobe).
 * @param isActive: A flag to indicate if a momentary/strobe is running.
 * @param isConfigured: A flag to indicate if a switch is configured.
 * @param hardwareInitialized: A flag to indicate if hardware is initialized.
 */
struct outputTracker_t {
    uint32_t     nextActionTime;        /**< Timestamp for the next state change */
    uint8_t      currentStep;           /**< Current step in a multi-stage pattern (strobe) */
    uint8_t      timer;                 /**< LEDC timer */
    bool         isActive;              /**< Flag to indicate if a momentary/strobe is running */
    bool         isConfigured;          /**< Flag to indicate if a switch is configured */
    bool         hardwareInitialized;   /**< Flag to indicate if hardware is initialized */
    bool         hasBeenSet;            /**< Flag to indicate if a switch has been set */
}; /* end struct outputTracker_t */

extern nodeInfo_t node; /**< Global node configuration */
extern outputTracker_t trackers[MAX_SUB_MODULES]; /**< Global output trackers */

#ifdef __cplusplus
}
#endif

/* ============================================================================
 *  NODE RELATED FUNCTIONS
 * ============================================================================ */ 

void initHardware();
void loadDefaults(uint16_t nodeType);
const uint16_t getConfigurationCRC(const nodeInfo_t& node);
const uint16_t getSubModuleCRC(const subModule_t& submod);
