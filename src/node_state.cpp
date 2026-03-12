#include "node_state.h"

#ifdef __cplusplus
extern "C" {
#endif


subModule_t* nodeGetSubmodule(const uint8_t sub_idx)
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

/**
 * @brief Return the number of sub-modules associated with this node
 *
 * This function returns the number of sub-modules associated with this node.
 * If the number of sub-modules exceeds the maximum allowed (MAX_SUB_MODULES),
 * the function will return MAX_SUB_MODULES.
 *
 * @return uint8_t The number of sub-modules associated with this node.
 */
uint8_t nodeGetSubmoduleCount(void)
{
    uint8_t count = node.subModCnt;

    if (count > MAX_SUB_MODULES)
        count = MAX_SUB_MODULES;

    return count;
}

#ifdef __cplusplus
}
#endif