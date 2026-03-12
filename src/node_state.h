#pragma once

#include <stddef.h>

#include "canbus_project.h"   // for node_t, subModule_t, etc.


/* ============================================================================
 *  CONSTANTS
 * ============================================================================ */ 


/* ============================================================================
 *  GLOBAL VARIABLES
 * ============================================================================ */ 

extern volatile bool FLAG_VALID_CONFIG;

struct outputTracker_t {
    uint32_t     nextActionTime;        /**< Timestamp for the next state change */
    uint8_t      currentStep;           /**< Current step in a multi-stage pattern (strobe) */
    uint8_t      timer;                 /**< LEDC timer */
    bool         isActive;              /**< Flag to indicate if a momentary/strobe is running */
    bool         isConfigured;          /**< Flag to indicate if a switch is configured */
    bool         hardwareInitialized;   /**< Flag to indicate if hardware is initialized */
    bool         hasBeenSet;            /**< Flag to indicate if a switch has been set */
}; /* end struct outputTracker_t */

extern nodeInfo_t node;                           /**< Store information about this node */
extern outputTracker_t trackers[MAX_SUB_MODULES]; /**< Track hardware output states */




#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  GLOBAL FUNCTIONS
 * ============================================================================ */ 

 /** @brief Send message to the CAN bus, lives in main.cpp */
void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc );
void updateSubmoduleRuntime(void);

/* ============================================================================
 *  NODE STATE API
 * ============================================================================ */ 

/* ============================================================================
 *  ACCESSOR FUNCTIONS
 * ============================================================================ */ 
/* producer specific accessor functions */
runTime_t*   producerGetRuntime(const uint8_t sub_idx);
uint8_t      producerGetFlags(const uint8_t sub_idx);
void         producerSetFlags(const uint8_t sub_idx, uint8_t flags);

/* generic accessor functions */
subModule_t* nodeGetSubmodule(const uint8_t sub_idx);
uint8_t      nodeGetSubmoduleCount(void);



#ifdef __cplusplus
}
#endif

/* ============================================================================
 *  NODE RELATED FUNCTIONS
 * ============================================================================ */ 

void initHardware();
void loadDefaults(uint16_t nodeType);
uint16_t getConfigurationCRC(const nodeInfo_t& node);
