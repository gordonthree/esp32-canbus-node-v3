#include "node_state.h"

producer_t* producerGetState(const uint8_t sub_idx) {
    return &node.subModule[sub_idx].producer;
}

producer_cfg_t* producerGetConfig(const uint8_t sub_idx) {
    return &node.subModule[sub_idx].producer_cfg;
}

uint8_t producerGetFlags(const uint8_t sub_idx) {
    return node.subModule[sub_idx].flags;   // or whatever field is relevant
}

void producerSetFlags(const uint8_t sub_idx, uint8_t flags) {
    node.subModule[sub_idx].flags = flags;
}

