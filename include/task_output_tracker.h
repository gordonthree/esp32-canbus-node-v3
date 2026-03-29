#pragma once

#include "node_state.h"
#include "personality_table.h"
#include "strobe_patterns.h"

#include <stdint.h>

/*
 * ==========================================================================
 * Constants and macros
 * =========================================================================
 */

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
struct outputTracker_t
{
    uint32_t nextActionTime;  /**< Timestamp for the next state change */
    uint8_t currentStep;      /**< Current step in a multi-stage pattern (strobe) */
    uint8_t timer;            /**< LEDC timer */
    bool isActive;            /**< Flag to indicate if a momentary/strobe is running */
    bool outputSupported;     /**< Flag to indicate if a out is supported */
    bool hardwareInitialized; /**< Flag to indicate if hardware is initialized */
    bool hasBeenSet;          /**< Flag to indicate if a switch has been set */
}; /* end struct outputTracker_t */

/*
 * ==========================================================================
 * Public API Functions
 * =========================================================================
 */

outputTracker_t *getOutputTracker(const uint8_t index);
outputTracker_t *getOutputTrackers(void);
void outputTrackerTick(void);
void outputTrackerInit(void);
void outputTrackerReset(const uint8_t index);
void outputTrackerActive(const uint8_t index, bool state);
void outputTrackerConfig(const uint8_t index, bool state);