#include "personality_table.h"

extern const personalityDef_t g_personalityTable[];
extern uint8_t g_personalityCount;

const personalityDef_t *getPersonality(uint8_t index)
{
    if (index >= g_personalityCount) {
        return NULL;
    }
    return &g_personalityTable[index];
}
