#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "cmsis_device.h"
#include "bootloader_config.h"


#define PORT_CAN_SILENT GPIOB
#define PIN_CAN_SILENT_INDEX 6
#define PORT_CAN_RX GPIOA
#define PIN_CAN_RX_INDEX 11
#define PORT_CAN_TX GPIOA
#define PIN_CAN_TX_INDEX 12


uint8_t board_get_product_name(uint8_t *product_name) {
    product_name[0] = 'h';
    product_name[1] = 'i';
    product_name[2] = '!';
    return 3u;
}


void board_get_hardware_version(uavcan_hardwareversion_t *hw_version) {
    uint32_t i;
    volatile uint8_t *stm32f302_uid = (volatile uint8_t*)0x1FFFF7ACu;

    hw_version->major = 1u;
    hw_version->minor = 0u;

    for (i = 0u; i < 12u; i++) {
        hw_version->unique_id[i] = stm32f302_uid[i];
    }
    for (; i < 16u; i++) {
        hw_version->unique_id[i] = 0u;
    }

    for (i = 0u; i < 255u; i++) {
        hw_version->certificate_of_authenticity[i] = 0;
    }

    hw_version->certificate_of_authenticity_length = 0u;
}


void board_initialize(void) {
    /*
    TODO: replace this with e.g. NuttX init code and compile-time options.

    We need to enable the CAN RX and TX pins, do whatever we need to take the
    CAN transceiver out of silent (if supported by the hardware), and ensure
    hardware is left in a safe state.
    */
    enum {
        GPIO_Speed_Level_1  = 1,
        GPIO_Speed_Level_2  = 2,
        GPIO_Speed_Level_3  = 3
    };
    enum {
        GPIO_OType_PP = 0,
        GPIO_OType_OD = 1
    };
    enum {
        GPIO_Mode_IN   = 0,
        GPIO_Mode_OUT  = 1,
        GPIO_Mode_AF   = 2,
        GPIO_Mode_AN   = 3
    };
    enum {
        GPIO_PuPd_NOPULL = 0,
        GPIO_PuPd_UP     = 1,
        GPIO_PuPd_DOWN   = 2
    };

    uint32_t offset;

    SystemInit();
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* CAN_DeInit(CAN1); */
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    /*
    Configure RX and TX pins with:
    GPIO_Mode_AF
    GPIO_Speed_Level_2
    GPIO_OType_PP
    GPIO_PuPd_NOPULL

    Configure PIN_CAN_SILENT as above but GPIO_Mode_OUT.
    */

    /* GPIO_Init(PORT_CAN_TX, PIN_CAN_TX); */
    offset = PIN_CAN_TX_INDEX * 2u;
    PORT_CAN_TX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_TX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_TX->OTYPER = (uint16_t)
        (PORT_CAN_TX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_TX_INDEX));
    PORT_CAN_TX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_TX_INDEX);
    PORT_CAN_TX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_TX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_TX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_TX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_RX, PIN_CAN_RX); */
    offset = PIN_CAN_RX_INDEX * 2u;
    PORT_CAN_RX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_RX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_RX->OTYPER = (uint16_t)
        (PORT_CAN_RX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_RX_INDEX));
    PORT_CAN_RX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_RX_INDEX);
    PORT_CAN_RX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_RX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_RX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_RX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_SILENT, PIN_CAN_SILENT); */
    offset = PIN_CAN_SILENT_INDEX * 2u;
    PORT_CAN_SILENT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_SILENT->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_SILENT->OTYPER = (uint16_t)
        (PORT_CAN_SILENT->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_SILENT_INDEX));
    PORT_CAN_SILENT->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_SILENT_INDEX);
    PORT_CAN_SILENT->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_SILENT->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_SILENT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_SILENT->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_PinAFConfig(PORT_CAN_RX, GPIO_PinSource11, GPIO_AF_9); */
    offset = (PIN_CAN_RX_INDEX & 7u) * 4u;
    PORT_CAN_RX->AFR[PIN_CAN_RX_INDEX >> 3u] &= ~(0xFu << offset);
    PORT_CAN_RX->AFR[PIN_CAN_RX_INDEX >> 3u] |= 0x9u << offset;

    /* GPIO_PinAFConfig(PORT_CAN_TX, GPIO_PinSource12, GPIO_AF_9); */
    offset = (PIN_CAN_TX_INDEX & 7u) * 4u;
    PORT_CAN_TX->AFR[PIN_CAN_TX_INDEX >> 3u] &= ~(0xFu << offset);
    PORT_CAN_TX->AFR[PIN_CAN_TX_INDEX >> 3u] |= 0x9u << offset;

    /* GPIO_ResetBits(PORT_CAN_SILENT, PIN_CAN_SILENT.GPIO_Pin); */
    PORT_CAN_SILENT->BRR = (1u << PIN_CAN_SILENT_INDEX);
}


void board_indicate_reset(void) {

}


void board_indicate_autobaud_start(void) {

}


void board_indicate_autobaud_end(void) {

}


void board_indicate_allocation_start(void) {

}


void board_indicate_allocation_end(void) {

}


void board_indicate_fw_update_start(void) {

}


void board_indicate_fw_update_erase_fail(void) {

}


void board_indicate_fw_update_invalid_response(void) {

}


void board_indicate_fw_update_timeout(void) {

}


void board_indicate_fw_update_invalid_crc(void) {

}


uint8_t board_get_wait_for_getnodeinfo_flag(void) {
    return 1u;
}
