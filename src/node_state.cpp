#include "node_state.h"
/** Runtime array accessor */
runTime_t* nodeGetRuntime(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].runTime;
}

/** Submodule array accessor */
subModule_t* nodeGetSubModule(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx];
}

/** Producer flags reader */
uint8_t* nodeGetProducerFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].producer_flags;  
}

/** Producer flags writer */
void nodeSetProducerFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].producer_flags = flags;
}

/** Submodule flags reader */
uint8_t* nodeGetSubmodFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].submod_flags;
}

/** Submodule flags writer */
void nodeSetSubmodFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].submod_flags = flags;
}

/** Router flags reader */
uint8_t* nodeGetRouterFlags(const uint8_t sub_idx) 
{
    return &node.subModule[sub_idx].router_flags;
}

/** Router flags writer */
void nodeSetRouterFlags(const uint8_t sub_idx, uint8_t flags) 
{
    node.subModule[sub_idx].router_flags = flags;
}

/** Return count of configured submodules */
const uint8_t nodeGetSubModuleCount(void)
{
    return sizeof(node.subModule) / sizeof(subModule_t);
}