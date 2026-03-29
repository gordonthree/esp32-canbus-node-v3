#pragma once

/* === Standard library includes === */
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "can_platform.h"      /* Brings in can_msg_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================= */
/* Functions */
/* ============================================================================= */

// void consumeMsg(can_msg_t *msgToConsume);


// void TaskConsumer(void *pvParameters);

void startTaskConsumer(void);

#ifdef __cplusplus
}
#endif
