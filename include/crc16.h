#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16_ccitt(const uint8_t *data, 
                     uint16_t length);

uint16_t crc16_ccitt_update(uint16_t crc, 
                            const uint8_t *data, 
                            uint16_t length);


#ifdef __cplusplus
}
#endif