#pragma once

#include <Arduino.h>

#include "canbus_project.h"   // for node_t, subModule_t, etc.
#include "submodule_types.h"   /**< Sub-module type definitions */
#include "personality_table.h" /**< Node and sub-module personality table */
// #include "consumer.h"          /**< CAN consumer routines and constants */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  GLOBAL VARIABLES, CONSTANTS and MACROS
 * ============================================================================ */ 


    #define SUBMODULE_INDEX_INVALID(idx) \
        ((idx) >= MAX_SUB_MODULES)


    #define PERSONALITY_INDEX_INVALID(idx) \
        ((idx) >= MAX_RUNTIME_PERSONALITIES)

/* ============================================================================ */
/*  NODE STATE API
 * ============================================================================ */

/* Node state accessor functions */
nodeInfo_t*    nodeGetInfo();
subModule_t*   nodeGetSubModule(const uint8_t sub_idx);
subModule_t*   nodeGetActiveSubModule(const uint8_t sub_idx);
runTime_t*     nodeGetRuntime(const uint8_t sub_idx);
uint8_t        nodeGetActiveSubModuleCount(void);
void           nodeIncSubModuleCount(void);
bool           nodeStateIsInitialized(void);


/* Utility functions */
uint32_t nodeGetNodeID(void);
uint16_t nodeGetNodeType(void);
void nodePrintStructInfo(void) ;
void nodeReadMacAddress(void); 
void nodeInit(void);

/* Personality functions */
const personalityDef_t* nodeGetPersonality(uint8_t personalityIndex);
const personalityDef_t *nodeGetActivePersonality(uint8_t personalityIndex);
uint8_t                 nodeGetActivePersonalityCount(void);
bool nodeIsValidPersonality(uint8_t personalityIndex);
bool nodeIsActivePersonality(uint8_t personalityIndex);

/* Validity accessor functions */
bool nodeIsValidSubmodule(uint8_t index);
bool nodeIsActiveSubmodule(uint8_t index);

/* Curated list functions */
uint8_t nodeGetInternalPeriodicProducers(uint8_t *outList, uint8_t max);

/* CRC accessor functions */
uint16_t       nodeGetCRC();
void nodeSetCRC(uint16_t crc);

/* Flags accessor functions */
uint8_t* nodeGetProducerFlags(const uint8_t sub_idx) ;
uint8_t* nodeGetSubmodFlags(const uint8_t sub_idx);
uint8_t* nodeGetRouterFlags(const uint8_t sub_idx);
void nodeSetProducerFlags(const uint8_t sub_idx, uint8_t flags);
void nodeSetSubmodFlags(const uint8_t sub_idx, uint8_t flags);
void nodeSetRouterFlags(const uint8_t sub_idx, uint8_t flags);

/** @brief Return true if the submodule has the input flag set */
inline bool nodeIsInputSubmodule(uint8_t sub_idx)
{
    const subModule_t* sub = nodeGetActiveSubModule(sub_idx);
    if (!sub)
        return false;

    return (sub->submod_flags & SUBMOD_FLAG_DIRECTION) != 0;  /**< true if input role */
}

inline bool nodeIsNetworkSubmodule(uint8_t sub_idx)
{
    /* get a safe pointer for the sub-module */
    const subModule_t* sub = nodeGetActiveSubModule(sub_idx);
    if (!sub)
        return false;

    /* get a safe pointer for the personality */
    const personalityDef_t* pers = nodeGetActivePersonality(sub->personalityIndex);
    if (!pers)
        return false;

    /* return true if the sub-module is a network node */
    return (pers->flags & BUILDER_FLAG_IS_NETWORK_NODE) != 0;
}

inline bool nodeIsInternalSubmodule(uint8_t sub_idx)
{
    /* get a safe pointer for the sub-module */
    const subModule_t* sub = nodeGetActiveSubModule(sub_idx);
    if (!sub)
        return false;

    /* get a safe pointer for the personality */
    const personalityDef_t* pers = nodeGetActivePersonality(sub->personalityIndex);
    if (!pers)
        return false;

    /* return true if the sub-module is internal */
    return (pers->flags & BUILDER_FLAG_IS_INTERNAL) != 0;
}


/* ============================================================================
 *  GLOBAL FUNCTIONS
 * ============================================================================ */ 

 /** @brief Send message to the CAN bus, lives in main.cpp */
// void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc );

/** pretty print the in-memory node config database */
void printNodeInfo(const nodeInfo_t* node);


/**
 * @brief Packs three uint8_t values representing red, green, and blue into a single uint32_t.
 *
 * The resulting uint32_t has the following format: 0xRRGGBB.
 *
 * @param r uint8_t value representing red (0-255)
 * @param g uint8_t value representing green (0-255)
 * @param b uint8_t value representing blue (0-255)
 * @return uint32_t containing the packed values
 */
static inline uint32_t packRgb(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)r << 16) |
           ((uint32_t)g << 8)  |
           (uint32_t)b;
}

/**
 * @brief Packs three values representing a strobe pattern into a single uint32_t.
 *
 * The resulting uint32_t has the following format: 0xPPSSO.
 *
 * @param patternId uint8_t value representing the strobe pattern ID (0-255)
 * @param step uint8_t value representing the current step in the pattern (0-255)
 * @param output bool value representing the current state of the strobe pattern (true/false)
 * @return uint32_t containing the packed values
 */
static inline uint32_t packStrobeState(uint8_t patternId, uint8_t step, bool output)
{
    return  ((uint32_t)patternId << 8) |
            ((uint32_t)step      << 1) |
            (output ? 1 : 0);
}


/* ============================================================================
 *  NODE STATE API
 * ============================================================================ */ 


// extern nodeInfo_t node; /* node is now private, owned by node_state */

#ifdef __cplusplus
}
#endif

/* ============================================================================
 *  Global Variables  
 * ============================================================================ */ 

// extern volatile uint32_t g_isr_counter;



/* ============================================================================
 * DECLARATIONS FOR NODE RELATED FUNCTIONS
 * ============================================================================ */ 

// void initHardware();
void loadDefaults(uint16_t nodeType);
uint32_t getEpochTime();

/** Inline function to validate sub-module index */
static inline bool isValidSubModuleIndex(uint8_t index) 
{ 
  return (index < MAX_SUB_MODULES); 
}

/**
 * @brief Validate that a personalityId refers to a user-defined template personality.
 * @note  NEW helper function, introduced explicitly.
 */
static inline bool isValidTemplatePersonality(uint8_t personalityId)
{
    for (uint8_t i = 0; i < g_TemplateCount; i++) {
        const personalityDef_t *p = &templateTable[i];

        /* Match personalityId and ensure it is a user-defined template */
        if (p->personalityId == personalityId &&
            (p->flags & BUILDER_FLAG_USER_DEFINED)) {
            return true;
        }
    }

    return false;
};