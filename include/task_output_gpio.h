#pragma once
#include "node_state.h"
#include "personality_table.h"

/* NEW: GPIO backend public API */
void gpioApplyState(uint8_t index, uint32_t state);
void gpioApplyPwmDuty(uint8_t index, uint32_t duty13bit);
void gpioApplyPwmFreq(uint8_t index, uint32_t freqHz);

/* Existing electrical helper */
void setOutput(subModule_t *sub, const personalityDef_t *p, bool desiredState);
void subOutHelper(uint8_t index, bool state);
