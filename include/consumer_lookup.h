#pragma once
#include <stdint.h>
#include "consumer_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t startId;                       /**< Inclusive start of ID range */
    uint16_t endId;                         /**< Inclusive end of ID range */
    void (*handler)(can_msg_t *msg);        /**< Handler for this range */
} ConsumerHandlerEntry;

extern const ConsumerHandlerEntry consumerHandlerTable[];
extern const uint8_t consumerHandlerTableCount;

#ifdef __cplusplus
}
#endif

