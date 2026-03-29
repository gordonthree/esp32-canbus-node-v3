#pragma once

#include <stdint.h>
#include "canbus_project.h"     /* shared project constants */
#include "can_platform.h"       /* for can_msg_t */
#include "task_output.h"

#ifdef __cplusplus
extern "C" {
#endif

/* === Constants and Macros === */




/** Forward declaration; actual definition lives in your CAN driver headers */
// typedef struct can_msg_t can_msg_t;

/* Subsystem handlers */
void handleOutputCommands(can_msg_t *msg);        /**< 0x110–0x1FF */
void handleDisplayCommands(can_msg_t *msg);       /**< 0x200–0x2FF */
void handleProducerConfig(can_msg_t *msg);        /**< 0x320–0x3FF */
void handleIdentityConfig(can_msg_t *msg);        /**< 0x400–0x4FF */
void handleNetworkConfig(can_msg_t *msg);         
void handleNvsConfig(can_msg_t *msg);             

/* Public dispatcher entry point */
void consumeMsg(can_msg_t *msg);

/* Public reset introduction sequence entry point */
void introResetSequence(void);

#ifdef __cplusplus
}
#endif

