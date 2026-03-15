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

/** Pretty print nodeInfo_t */
/**
 * @brief Prints a pretty formatted version of the nodeInfo_t structure to the Serial console.
 *
 * @param node Pointer to the nodeInfo_t structure to print.
 */
void printNodeInfo(const nodeInfo_t* node) 
{
  Serial.println("\n--------- nodeInfo_t ---------");
  Serial.printf("NodeInfo:\n");
  Serial.printf("  nodeID: 0x%08X\n", node->nodeID);
  Serial.printf("  nodeTypeMsg: 0x%04X\n", node->nodeTypeMsg);
  Serial.printf("  nodeTypeDLC: %d\n", node->nodeTypeDLC);
  Serial.printf("  subModCnt: %d\n", node->subModCnt);

  Serial.printf("  subModule array:\n");
  for (uint8_t i = 0; i < node->subModCnt; i++) {
    Serial.printf("    subModule[%d]:\n", i);
    Serial.printf("      personalityId: %d\n", node->subModule[i].personalityId);
    Serial.printf("      introMsgId: 0x%04X\n", node->subModule[i].introMsgId);
    Serial.printf("      introMsgDLC: %d\n", node->subModule[i].introMsgDLC);
    Serial.printf("      runTime:\n");
    Serial.printf("        last_change_ms: %d\n", node->subModule[i].runTime.last_change_ms);
    Serial.printf("        adc_value: %d\n", node->subModule[i].runTime.adc_value);
    Serial.printf("        state: %d\n", node->subModule[i].runTime.state);
    Serial.printf("        last_hardware_output: %d\n", node->subModule[i].runTime.last_hardware_output);
    Serial.printf("        kind: %d\n", node->subModule[i].runTime.kind);
    Serial.printf("        valueSource: %d\n", node->subModule[i].runTime.valueSource);
    Serial.printf("        period_ms: %d\n", node->subModule[i].runTime.period_ms);
    Serial.printf("        last_published_value: %d\n", node->subModule[i].runTime.last_published_value);
    // You can continue this pattern for each field in the subModule_t structure
  }
  Serial.println("--------- nodeInfo_t ---------\n");

}