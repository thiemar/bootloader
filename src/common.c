#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "common.h"
#include "crc.h"


uint8_t common_is_valid(void) {
    uint64_t crc;
    bootloader_app_common_t common;
    size_t i;

    common_read(&common);

    crc = CRC64_INITIAL;
    for (i = 0u; i < sizeof(uint32_t) * 4u; i++) {
        crc = crc64_add(crc, ((uint8_t*)(&common.signature))[i]);
    }
    crc ^= CRC64_OUTPUT_XOR;

    if (crc == common.crc) {
        return 1u;
    } else {
        return 0u;
    }
}


void common_read(bootloader_app_common_t *common) {
    common->signature = CAN1->sFilterRegister[0].FR1;
    common->bus_speed = CAN1->sFilterRegister[0].FR2;
    common->node_id = CAN1->sFilterRegister[1].FR1;
    common->uptime = common->app_progress = CAN1->sFilterRegister[1].FR2;
    common->crc = (uint64_t)CAN1->sFilterRegister[2].FR1 |
                  ((uint64_t)CAN1->sFilterRegister[2].FR2 << 32u);
}


void common_write(bootloader_app_common_t *common) {
    CAN1->FMR |= 1u;
    CAN1->sFilterRegister[0].FR1 = BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE;
    CAN1->sFilterRegister[0].FR2 = common->bus_speed;
    CAN1->sFilterRegister[1].FR1 = common->node_id;
    CAN1->sFilterRegister[1].FR2 = common->uptime;
    CAN1->sFilterRegister[2].FR1 = (uint32_t)common->crc;
    CAN1->sFilterRegister[2].FR2 = (uint32_t)(common->crc >> 32u);
    CAN1->FMR &= ~1u;
}


void common_invalidate(void) {
    CAN1->FMR |= 1u;
    CAN1->sFilterRegister[2].FR1 = CAN1->sFilterRegister[2].FR2 = 0u;
    CAN1->FMR &= ~1u;
}
