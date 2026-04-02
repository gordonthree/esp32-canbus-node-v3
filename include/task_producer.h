#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void startTaskProducer(void);
void attachGpioIsrInit(void);

#ifdef __cplusplus
}
#endif
