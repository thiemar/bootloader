#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "bootloader_config.h"
#include "uavcan.h"


#define CAN_1MBAUD_SJW 0
#define CAN_1MBAUD_BS1 6u
#define CAN_1MBAUD_BS2 0
#define CAN_1MBAUD_PRESCALER 4u

#define CAN_500KBAUD_SJW 0
#define CAN_500KBAUD_BS1 6u
#define CAN_500KBAUD_BS2 0
#define CAN_500KBAUD_PRESCALER 8u

#define CAN_250KBAUD_SJW 0
#define CAN_250KBAUD_BS1 6u
#define CAN_250KBAUD_BS2 0
#define CAN_250KBAUD_PRESCALER 16u

#define CAN_125KBAUD_SJW 0
#define CAN_125KBAUD_BS1 6u
#define CAN_125KBAUD_BS2 0
#define CAN_125KBAUD_PRESCALER 32u

/* Number of CPU cycles for a single bit time at the supported speeds */
#define CAN_1MBAUD_BIT_CYCLES 72u
#define CAN_500KBAUD_BIT_CYCLES 144u
#define CAN_250KBAUD_BIT_CYCLES 288u
#define CAN_125KBAUD_BIT_CYCLES 576u

/*
Number of CPU cycles for which we should listen for CAN frames during each
auto-bauding stage. Should correspond to 1.1 s.
*/
#define CAN_AUTOBAUD_CYCLES (72000000u + 7200000u)

/* 1 Mbaud from 72 MHz system clock */
#define CAN_INAK_TIMEOUT 100000
#define CAN_REQUEST_TIMEOUT 1000000


typedef enum {
    CAN_Mode_Normal = 0,
    CAN_Mode_LoopBack = 1,
    CAN_Mode_Silent = 2,
    CAN_Mode_Silent_LoopBack = 3
} CAN_Mode;


extern volatile uint8_t g_bootloader_tboot_timeout;


can_error_t uavcan_init(can_speed_t speed) {
    /*
    TODO: use full-word writes to reduce the number of loads/stores.

    Also consider filter use -- maybe set filters for all the message types
    we want.
    */

    /* CAN setup */
    uint32_t timer;

    /* CAN_DeInit(CAN1); */
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    /* CAN_Init(CAN1, &config); */
    CAN1->MCR = (CAN1->MCR & (~(uint32_t)CAN_MCR_SLEEP)) | CAN_MCR_INRQ;

    timer = CAN_INAK_TIMEOUT;
    while (!(CAN1->MSR & CAN_MSR_INAK) && timer) {
        timer--;
    }

    if (!timer) {
        /*
        Initialization failed, not much we can do now other than try a normal
        startup.
        */
        return CAN_ERROR;
    }

    CAN1->MCR &= ~(uint32_t)(CAN_MCR_TTCM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);
    CAN1->MCR |= (CAN_MCR_ABOM | CAN_MCR_AWUM);
    if (speed == CAN_1MBAUD) {
        CAN1->BTR =
            (CAN_Mode_Normal << 30) |
            (CAN_1MBAUD_SJW << 24) |
            (CAN_1MBAUD_BS1 << 16) |
            (CAN_1MBAUD_BS2 << 20) |
            (CAN_1MBAUD_PRESCALER - 1);
    } else if (speed == CAN_500KBAUD) {
        CAN1->BTR =
            (CAN_Mode_Normal << 30) |
            (CAN_500KBAUD_SJW << 24) |
            (CAN_500KBAUD_BS1 << 16) |
            (CAN_500KBAUD_BS2 << 20) |
            (CAN_500KBAUD_PRESCALER - 1);
    } else if (speed == CAN_250KBAUD) {
        CAN1->BTR =
            (CAN_Mode_Normal << 30) |
            (CAN_250KBAUD_SJW << 24) |
            (CAN_250KBAUD_BS1 << 16) |
            (CAN_250KBAUD_BS2 << 20) |
            (CAN_250KBAUD_PRESCALER - 1);
    } else if (speed == CAN_125KBAUD) {
        CAN1->BTR =
            (CAN_Mode_Normal << 30) |
            (CAN_125KBAUD_SJW << 24) |
            (CAN_125KBAUD_BS1 << 16) |
            (CAN_125KBAUD_BS2 << 20) |
            (CAN_125KBAUD_PRESCALER - 1);
    } else {
        return CAN_ERROR;
    }

    /* Leave initialization mode */
    CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;

    timer = CAN_INAK_TIMEOUT;
    while ((CAN1->MSR & CAN_MSR_INAK) && timer) {
        timer--;
    }
    if (!timer) {
        return CAN_ERROR;
    }

    /* CAN filter initialization -- accept everything */
    CAN1->FMR |= 1u;
    CAN1->FA1R &= ~1u;
    CAN1->FS1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FM1R &= ~1u;
    CAN1->FFA1R &= ~1u;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~1u;

    return CAN_OK;
}


void uavcan_tx(
    uint32_t message_id,
    size_t length,
    const uint32_t *message
) {
    /* Just block while waiting for the mailbox */
    while (!(CAN1->TSR & CAN_TSR_TME0));

    CAN1->sTxMailBox[0].TIR = 0;
    CAN1->sTxMailBox[0].TIR |= (message_id << 3u) | 0x4u;
    CAN1->sTxMailBox[0].TDTR = length;
    CAN1->sTxMailBox[0].TDLR = message[0];
    CAN1->sTxMailBox[0].TDHR = message[1];
    CAN1->sTxMailBox[0].TIR |= 1u;
}


uint8_t uavcan_rx(
    uint32_t *out_message_id,
    size_t *out_length,
    uint32_t *out_message
) {
    uint8_t fifo;

    /* Check if a message is pending */
    if (CAN1->RF0R & 3u) {
        fifo = 0;
    } else if (CAN1->RF1R & 3u) {
        fifo = 1;
    } else {
        return 0;
    }

    /* If so, process it */
    *out_message_id = CAN1->sFIFOMailBox[fifo].RIR >> 3u;
    *out_length = CAN1->sFIFOMailBox[fifo].RDTR & 0xFu;
    out_message[0] = CAN1->sFIFOMailBox[fifo].RDLR;
    out_message[1] = CAN1->sFIFOMailBox[fifo].RDHR;

    /* Release the message from the receive FIFO */
    if (fifo == 0) {
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    } else {
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }

    return 1;
}



can_error_t uavcan_autobaud(void) {
    can_speed_t current_speed;
    can_error_t status;
    uint32_t last_cyccnt, bit_cycles, current_speed_bit_cycles, last_msr;

    /*
    First, try to initialize the CAN interface with the lowest possible speed.

    Depending on bus conditions it's possible this will fail, as the
    controller requires at least 11 consecutive recessive bits to complete
    initialization. If that fails, we know that the bus is running (much)
    faster than the current speed, so increase it and try again.
    */
    current_speed = CAN_125KBAUD;
    current_speed_bit_cycles = CAN_125KBAUD_BIT_CYCLES -
                               (CAN_125KBAUD_BIT_CYCLES >> 2u);
    do {
        status = uavcan_init(current_speed);
        if (status != CAN_OK) {
            current_speed++;
            current_speed_bit_cycles >>= 1u;
        }
    } while (current_speed <= CAN_1MBAUD && status != CAN_OK);
    if (status != CAN_OK) {
        return status;
    }


    /* Wait for the first transition */
    last_msr = CAN1->MSR;
    while (!g_bootloader_tboot_timeout &&
            ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
    last_cyccnt = DWT->CYCCNT;

    /* Start the bit timing loop */
    do {
        last_msr = CAN1->MSR;
        /* Measure the duration of the next transition */
        while (!g_bootloader_tboot_timeout &&
            ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
        bit_cycles = DWT->CYCCNT - last_cyccnt;
        last_cyccnt = DWT->CYCCNT;

        /*
        If the number of cycles for the last bit transition was smaller than
        the duration of a bit at the current speed, change to the next higher
        speed.
        */
        if (!g_bootloader_tboot_timeout &&
                bit_cycles < current_speed_bit_cycles) {
            current_speed++;
            current_speed_bit_cycles >>= 1u;
            status = uavcan_init(current_speed);
            if (status != CAN_OK) {
                return status;
            }

            /* Wait for the next transition */
            last_msr = CAN1->MSR;
            while (!g_bootloader_tboot_timeout &&
                   ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
            last_cyccnt = DWT->CYCCNT;
        }
    } while (!g_bootloader_tboot_timeout && current_speed < CAN_1MBAUD &&
             !(CAN1->RF0R & 3u) && !(CAN1->RF1R & 3u));

    /* Did we time out or get a message? */
    if (g_bootloader_tboot_timeout) {
        return CAN_ERROR;
    } else {
        return CAN_OK;
    }
}


size_t uavcan_pack_nodestatus(
    uint32_t *data,
    const uavcan_nodestatus_t *payload
) {
    /* No need to clear the top 4 bits of uptime_sec */
    data[0] = (payload->uptime_sec & 0x00FFFFFF) |
              (payload->status_code << 24u) |
              ((payload->uptime_sec & 0x0F000000) << 4u);
    data[1] = payload->vendor_specific_status_code;
    return 6u;
}


size_t uavcan_pack_dynamicnodeidallocation(
    uint32_t *data,
    const uavcan_dynamicnodeidallocation_t *payload
) {
    data[0] = payload->short_unique_id & 0xFFFFFFFFu;
    data[1] = ((payload->short_unique_id >> 32u) & 0x00FFFFFFu) |
              ((payload->node_id & 0x7Fu) << 24u) |
              ((payload->short_unique_id >> 25u) & 0x80000000u);
    return 8u;
}


void uavcan_unpack_dynamicnodeidallocation(
    uavcan_dynamicnodeidallocation_t *payload,
    const uint32_t *data
) {
    payload->short_unique_id = data[0] |
                               ((uint64_t)(data[1] & 0x00FFFFFFu) << 32u) |
                               ((uint64_t)(data[1] & 0x80000000u) << 25u);
    payload->node_id = (uint8_t)((data[1] & 0x7FFFFFFFu) >> 24u);
}


size_t uavcan_pack_logmessage(
    uint32_t *data,
    const uavcan_logmessage_t *payload
) {
    data[0] = ((payload->level << 5u) | UAVCAN_LOGMESSAGE_SOURCE_LEN) |
              ((UAVCAN_LOGMESSAGE_SOURCE & 0x00FFFFFFu) << 8u);
    data[1] = (UAVCAN_LOGMESSAGE_SOURCE >> 24u) |
              (payload->message[0] << 8u) |
              (payload->message[1] << 16u);
    return 7u;
}


uint32_t uavcan_make_frame_id(const uavcan_frame_id_t *metadata) {
    return (metadata->transfer_id & 0x7u) |
           ((metadata->last_frame ? 1u : 0u) << 3u) |
           ((metadata->frame_index & 0x3Fu) << 4u) |
           ((metadata->source_node_id & 0x7Fu) << 10u) |
           ((metadata->transfer_type & 0x3u) << 17u) |
           ((metadata->data_type_id & 0x3FFu) << 19u);
}


void uavcan_parse_frame_id(uavcan_frame_id_t *metadata, uint32_t frame_id) {
    metadata->transfer_id = frame_id & 0x7u;
    metadata->last_frame = (frame_id & 0x8u) ? 1u : 0u;
    metadata->frame_index = (frame_id >> 4u) & 0x3Fu;
    metadata->source_node_id = (frame_id >> 10u) & 0x7Fu;
    metadata->transfer_type = (frame_id >> 17u) & 0x3u;
    metadata->data_type_id = (frame_id >> 19u) & 0x3FFu;
}
