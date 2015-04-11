#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "bootloader_config.h"
#include "can.h"
#include "crc.h"


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

/* 1 Mbaud from 72 MHz system clock */
#define CAN_INAK_TIMEOUT 100000


extern volatile uint8_t g_bootloader_tboot_expired;


can_error_t can_init(can_speed_t speed, uint8_t silent) {
    /*
    TODO: use full-word writes to reduce the number of loads/stores.

    Also consider filter use -- maybe set filters for all the message types
    we want.
    */
    enum {
        CAN_Mode_Normal = 0,
        CAN_Mode_LoopBack = 1,
        CAN_Mode_Silent = 2,
        CAN_Mode_Silent_LoopBack = 3
    };

    uint32_t timeout;

    /* CAN_Init(CAN1, &config); */
    CAN1->MCR = (CAN1->MCR & (~(uint32_t)CAN_MCR_SLEEP)) | CAN_MCR_INRQ;

    timeout = CAN_INAK_TIMEOUT;
    while (!(CAN1->MSR & CAN_MSR_INAK) && timeout) {
        timeout--;
    }

    if (!timeout) {
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
            ((silent ? CAN_Mode_Silent : CAN_Mode_Normal) << 30u) |
            (CAN_1MBAUD_SJW << 24u) |
            (CAN_1MBAUD_BS1 << 16u) |
            (CAN_1MBAUD_BS2 << 20u) |
            (CAN_1MBAUD_PRESCALER - 1u);
    } else if (speed == CAN_500KBAUD) {
        CAN1->BTR =
            ((silent ? CAN_Mode_Silent : CAN_Mode_Normal) << 30u) |
            (CAN_500KBAUD_SJW << 24u) |
            (CAN_500KBAUD_BS1 << 16u) |
            (CAN_500KBAUD_BS2 << 20u) |
            (CAN_500KBAUD_PRESCALER - 1u);
    } else if (speed == CAN_250KBAUD) {
        CAN1->BTR =
            ((silent ? CAN_Mode_Silent : CAN_Mode_Normal) << 30u) |
            (CAN_250KBAUD_SJW << 24u) |
            (CAN_250KBAUD_BS1 << 16u) |
            (CAN_250KBAUD_BS2 << 20u) |
            (CAN_250KBAUD_PRESCALER - 1u);
    } else if (speed == CAN_125KBAUD) {
        CAN1->BTR =
            ((silent ? CAN_Mode_Silent : CAN_Mode_Normal) << 30u) |
            (CAN_125KBAUD_SJW << 24u) |
            (CAN_125KBAUD_BS1 << 16u) |
            (CAN_125KBAUD_BS2 << 20u) |
            (CAN_125KBAUD_PRESCALER - 1u);
    } else {
        return CAN_ERROR;
    }

    /* Leave initialization mode */
    CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;

    timeout = CAN_INAK_TIMEOUT;
    while ((CAN1->MSR & CAN_MSR_INAK) && timeout) {
        timeout--;
    }
    if (!timeout) {
        return CAN_ERROR;
    }

    /*
    CAN filter initialization -- accept everything on RX FIFO 0, and only
    GetNodeInfo requests on RX FIFO 1.
    */
    CAN1->FMR |= 1u; /* Start init mode */
    CAN1->FA1R &= ~3u; /* Disable filters 0 and 1 */
    CAN1->FS1R |= 3u; /* Enable 32-bit mode for filters 0 and 1 */
    /* Filter 0 masks -- data type ID 551 only */
    CAN1->sFilterRegister[0].FR1 = UAVCAN_GETNODEINFO_DTID << 22u;
    CAN1->sFilterRegister[0].FR2 = 0xFFC00000u; /* Top 10 bits of ID */
    /* Filter 1 masks -- everything is don't-care */
    CAN1->sFilterRegister[1].FR1 = 0;
    CAN1->sFilterRegister[1].FR2 = 0;
    CAN1->FM1R &= ~3u; /* Mask mode for filters 0 and 1 */
    CAN1->FFA1R |= 1u; /* FIFO 1 for filter 0 */
    CAN1->FFA1R &= ~2u; /* FIFO 0 for filter 1 */
    CAN1->FA1R |= 3u; /* Enable filters 0 and 1 */
    CAN1->FMR &= ~1u; /* Leave init mode */

    return CAN_OK;
}


void can_tx(
    uint32_t message_id,
    size_t length,
    const uint8_t *message,
    uint8_t mailbox
) {
    uint32_t data[2], cyccnt, mask, i;

    for (i = 0u; i < 8u; i++) {
        ((uint8_t*)data)[i] = message[i];
    }

    switch (mailbox) {
        case 0u:
            mask = CAN_TSR_TME0;
            break;
        case 1u:
            mask = CAN_TSR_TME1;
            break;
        case 2u:
        default:
            mask = CAN_TSR_TME2;
            break;
    }

    /*
    Just block while waiting for the mailbox. Give it an extra 0.75 ms per
    frame to avoid an issue I'm seeing with packets going missing on a USBtin.
    */
    cyccnt = DWT->CYCCNT;
    while (!(CAN1->TSR & mask) || DWT->CYCCNT - cyccnt < 48000u);

    CAN1->sTxMailBox[mailbox].TIR = 0;
    CAN1->sTxMailBox[mailbox].TIR |= (message_id << 3u) | 0x4u;
    CAN1->sTxMailBox[mailbox].TDTR = length;
    CAN1->sTxMailBox[mailbox].TDLR = data[0];
    CAN1->sTxMailBox[mailbox].TDHR = data[1];
    CAN1->sTxMailBox[mailbox].TIR |= 1u;
}


uint8_t can_rx(
    uint32_t *out_message_id,
    size_t *out_length,
    uint8_t *out_message,
    uint8_t fifo
) {
    uint32_t data[2], i;

    /* Check if a message is pending */
    if (fifo == 0u && !(CAN1->RF0R & 3u)) {
        return 0u;
    } else if (fifo == 1u && !(CAN1->RF1R & 3u)) {
        return 0u;
    } else if (fifo > 1u) {
        return 0u;
    }

    /* If so, process it */
    *out_message_id = CAN1->sFIFOMailBox[fifo].RIR >> 3u;
    *out_length = CAN1->sFIFOMailBox[fifo].RDTR & 0xFu;
    data[0] = CAN1->sFIFOMailBox[fifo].RDLR;
    data[1] = CAN1->sFIFOMailBox[fifo].RDHR;

    /* Release the message from the receive FIFO */
    if (fifo == 0u) {
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    } else {
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }

    for (i = 0u; i < 8u; i++) {
        out_message[i] = ((uint8_t*)data)[i];
    }

    return 1u;
}


can_error_t can_autobaud(void) {
    can_speed_t measured_speed;
    uint32_t last_cyccnt, bit_cycles, measured_speed_bit_cycles, last_msr,
             overrun;

    /*
    First, try to initialize the CAN interface with the lowest possible speed.

    Depending on bus conditions it's possible this will fail, as the
    controller requires at least 11 consecutive recessive bits to complete
    initialization. If that fails, we know that the bus is running (much)
    faster than the current speed, so increase it and try again.
    */
    measured_speed = CAN_UNKNOWN;
    measured_speed_bit_cycles = (CAN_125KBAUD_BIT_CYCLES -
                                 (CAN_125KBAUD_BIT_CYCLES >> 2u)) << 1u;

    /* Wait for the first transition */
    last_msr = CAN1->MSR;
    while (~(CAN1->MSR ^ last_msr) & CAN_MSR_RX) {
        if (g_bootloader_tboot_expired) {
            goto error;
        }
    }
    last_cyccnt = SysTick->VAL;

    /* Start the bit timing loop */
    while (1) {
        last_msr = CAN1->MSR;
        /* Measure the duration of the next transition */
        while (~(CAN1->MSR ^ last_msr) & CAN_MSR_RX) {
            if (g_bootloader_tboot_expired) {
                goto error;
            } else if (SysTick->VAL > last_cyccnt) {
                overrun = 1u;
            }
        }

        bit_cycles = last_cyccnt - SysTick->VAL + 20u;
        last_cyccnt = SysTick->VAL;

        /* Ignore any bits during which SysTick wrapped around */
        if (overrun) {
            overrun = 0u;
            continue;
        }

        /* If we received a frame at the current speed, it must be OK */
        if ((CAN1->RF0R | CAN1->RF1R) & 3u) {
            break;
        }

        /*
        If the number of cycles for the last bit transition was smaller than
        the duration of a bit at the current speed, change to the next higher
        speed.
        */
        if (bit_cycles < measured_speed_bit_cycles) {
            /* Increase speed to match the bit timing */
            while (bit_cycles < measured_speed_bit_cycles &&
                   measured_speed < CAN_1MBAUD) {
                measured_speed++;
                measured_speed_bit_cycles >>= 1u;
            }

            (void)can_init(measured_speed, 1u);

            /* Wait for the next transition */
            last_msr = CAN1->MSR;
            while (~(CAN1->MSR ^ last_msr) & CAN_MSR_RX) {
                if (g_bootloader_tboot_expired) {
                    goto error;
                }
            }
            last_cyccnt = SysTick->VAL;
        }
    }

    if (measured_speed > CAN_UNKNOWN) {
        return can_init(measured_speed, 0u);
    }

error:
    return CAN_ERROR;
}
