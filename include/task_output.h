/* ========================================================================= */
/* Includes */
/* ========================================================================= */

#include <Arduino.h>

#include "task_twai.h"         /**< my TWAI task */
#include "canbus_project.h"    /**< my various CAN functions and structs */
#include "can_router.h"        /**< CAN routing (subcriber) routines and constants */
#include "node_state.h"        /**< Node and sub-module state table */
#include "freertos.h"          /**< FreeRTOS definitions */

/* ========================================================================= */
/* Constants and macros                                       */
/* ========================================================================= */





/* ========================================================================= */
/* Public Variables                                       */
/* ========================================================================= */


/* ========================================================================= */
/* Public API Functions                                       */
/* ========================================================================= */
void enqueueOutputCmd(OutputCmdType_t type,
                      uint8_t        index,
                      uint32_t       param1);

void startTaskOutput();

void setOutput(subModule_t *sub,
                             const personalityDef_t* p,
                             bool desiredState);

void subOutHelper(const uint8_t index, const bool state);

bool blockLocalMsg(const can_msg_t *msg);