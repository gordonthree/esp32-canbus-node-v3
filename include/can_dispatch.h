#pragma once

/* ========================================================================= */
/* Includes */
/* ========================================================================= */

#include <stdbool.h>
#include <stdint.h>

#include "canbus_project.h"    /**< my various CAN functions and structs */


/* ========================================================================= */
/* Constants and macros                                       */
/* ========================================================================= */

#define SUBMOD_PART_B_FLAG     (0x80U)
#define CRC_INVALID_CONFIG     (0xFFFF)


/* ========================================================================= */
/* Public Variables                                       */
/* ========================================================================= */


/* ========================================================================= */
/* Public API Functions                                       */
/* ========================================================================= */

void canDispatchStartIntroduction();
void canDispatchProcessTimers();
bool canDispatchHasPendingNetworkData();
void prepareMessage(uint16_t msgid, uint8_t *data, uint8_t dlc);
void sendRouteList();
void sendIntroduction(int msgPtr);
