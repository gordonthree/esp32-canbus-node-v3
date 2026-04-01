#pragma once

#include "canbus_project.h"    /**< my various CAN functions and structs */
#include "freertos.h"          /*<< RTOS task handle declarations */


/* ========================================================================= */
/* FreeRTOS TWAI Task Public Variables                                       */
/* ========================================================================= */




/* ========================================================================= 
  FreeRTOS TWAI Task Public API Functions
  ========================================================================= */

/* expose task to start the TWAI task */
void startTaskTWAI();

/* Getters to on TWAI state */

bool twaiIsDriverInstalled();
unsigned long twaiLastErrorTime();

bool twaiIsSuspended();
void twaiSetSuspended(bool suspended);

void canEnqueueMessage(
    uint16_t msgid, 
    const uint8_t *data, 
    uint8_t dlc
  );
  
void canSendUint32(
    const uint32_t nodeID, 
    const uint32_t bigNumber, 
    const uint32_t canMsgId, 
    const uint8_t dlc
  );