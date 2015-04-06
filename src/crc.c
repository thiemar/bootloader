#include <stdint.h>
#include "crc.h"


/*
CRC-16-CCITT
Initial value: 0xFFFF
Poly: 0x1021
Reverse: no
Output xor: 0
*/
uint16_t crc16_add(uint16_t crc, uint8_t value) {
    const uint16_t poly = 0x1021u;
    crc ^= (uint16_t)((uint16_t)value << 8u);
    for (uint32_t i = 0; i < 8; i++) {
        if (crc & (1u << 15u)) {
            crc = (uint16_t)((crc << 1u) ^ poly);
        } else {
            crc = (uint16_t)(crc << 1u);
        }
    }
    return crc;
}


/*
CRC-64-WE
Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
Initial value: 0xFFFFFFFFFFFFFFFF
Poly: 0x42F0E1EBA9EA3693
Reverse: no
Output xor: 0xFFFFFFFFFFFFFFFF
Check: 0x62EC59E3F1A4F00A
*/
uint64_t crc64_add(uint64_t crc, uint8_t value) {
    const uint64_t poly = 0x42F0E1EBA9EA3693ull;
    crc ^= (uint64_t)value << 56u;
    for (uint32_t i = 0; i < 8; i++) {
        if (crc & (1ull << 63u)) {
            crc = (uint64_t)(crc << 1u) ^ poly;
        } else {
            crc = (uint64_t)(crc << 1u);
        }
    }
    return crc;
}
