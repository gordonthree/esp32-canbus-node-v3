#include "crc16.h"


/**
 * @brief Calculates a 16-bit CRC for the node configuration.
 * @details Uses the ESP32 ROM CRC16 implementation (CCITT).
 * @param node Reference to the canNodeInfo struct.
 * @return uint16_t The calculated checksum.
 */
uint16_t crc16_ccitt(const uint8_t* data, uint16_t length) 
{
  uint16_t crc = 0xFFFF; // Initial value
  while (length--) {
    crc ^= (uint16_t)*data++ << 8;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021; // Polynomial
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint16_t crc16_ccitt_update(uint16_t crc, 
                            const uint8_t *data, 
                            uint16_t length)
{
    while (length--) {
        crc ^= (uint16_t)(*data++ << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
