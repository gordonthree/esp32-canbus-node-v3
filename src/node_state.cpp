#include "node_state.h"

#ifdef __cplusplus
extern "C" {
#endif


subModule_t* producerGetSubmodule(const uint8_t sub_idx)
{
    if (sub_idx >= MAX_SUB_MODULES)
        return NULL;

    return &node.subModule[sub_idx];
}

runTime_t* producerGetRuntime(const uint8_t sub_idx) 
{
    if (sub_idx >= MAX_SUB_MODULES)
        return NULL;
    return &node.subModule[sub_idx].runTime;
}


uint8_t producerGetFlags(const uint8_t sub_idx) 
{
    if (sub_idx >= MAX_SUB_MODULES)
        return SUBMOD_FLAG_NONE;
    return node.subModule[sub_idx].producer_flags;   /* access producer flags only */
}

void producerSetFlags(const uint8_t sub_idx, const uint8_t flags) 
{
    if (sub_idx >= MAX_SUB_MODULES)
        return;
    node.subModule[sub_idx].producer_flags = flags;   /* update producer flags only */
}

#ifdef __cplusplus
}
#endif