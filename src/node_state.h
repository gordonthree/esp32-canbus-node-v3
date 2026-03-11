#pragma once

#include "canbus_project.h"   // for node_t, subModule_t, etc.

#ifdef __cplusplus
extern "C" {
#endif


extern volatile bool FLAG_VALID_CONFIG;

/* ============================================================================
 *  GLOBAL FUNCTIONS
 * ============================================================================ */ 

 /** @brief Send message to the CAN bus, lives in main.cpp */
extern void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc );


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

extern nodeInfo_t node;
extern outputTracker_t trackers[MAX_SUB_MODULES];

#ifdef __cplusplus
}
#endif

/* ============================================================================
 *  NODE RELATED FUNCTIONS
 * ============================================================================ */ 

void initHardware();
void loadDefaults(uint16_t nodeType);
uint16_t getConfigurationCRC(const nodeInfo_t& node);
