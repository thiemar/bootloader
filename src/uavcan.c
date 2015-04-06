#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "bootloader_config.h"
#include "uavcan.h"
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
#define CAN_REQUEST_TIMEOUT 1000000


extern volatile uint8_t g_bootloader_tboot_expired;
extern volatile uint32_t g_bootloader_uptime;

static can_error_t uavcan_rx_multiframe_(
    uint8_t node_id,
    uavcan_frame_id_t *message_id,
    uint8_t *message,
    size_t *message_length,
    uint16_t initial_crc,
    uint32_t timeout_cycles
);


can_error_t uavcan_init(can_speed_t speed) {
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


void uavcan_tx(
    const uavcan_frame_id_t *message_id,
    size_t length,
    const uint8_t *message,
    uint8_t mailbox
) {
    uint32_t frame_id, rdhr, rdlr, cyccnt, mask;
    frame_id = uavcan_make_frame_id(message_id);

    ((uint8_t*)&rdlr)[0] = message[0];
    ((uint8_t*)&rdlr)[1] = message[1];
    ((uint8_t*)&rdlr)[2] = message[2];
    ((uint8_t*)&rdlr)[3] = message[3];

    ((uint8_t*)&rdhr)[0] = message[4];
    ((uint8_t*)&rdhr)[1] = message[5];
    ((uint8_t*)&rdhr)[2] = message[6];
    ((uint8_t*)&rdhr)[3] = message[7];

    if (mailbox == 0u) {
        mask = CAN_TSR_TME0;
    } else if (mailbox == 1u) {
        mask = CAN_TSR_TME1;
    } else if (mailbox == 2u) {
        mask = CAN_TSR_TME2;
    } else {
        return;
    }

    /*
    Just block while waiting for the mailbox. Give it an extra 0.75 ms per
    frame to avoid an issue I'm seeing with packets going missing on a USBtin.
    */
    cyccnt = DWT->CYCCNT;
    while (DWT->CYCCNT - cyccnt < 48000u);
    while (!(CAN1->TSR & mask));

    CAN1->sTxMailBox[mailbox].TIR = 0;
    CAN1->sTxMailBox[mailbox].TIR |= (frame_id << 3u) | 0x4u;
    CAN1->sTxMailBox[mailbox].TDTR = length;
    CAN1->sTxMailBox[mailbox].TDLR = rdlr;
    CAN1->sTxMailBox[mailbox].TDHR = rdhr;
    CAN1->sTxMailBox[mailbox].TIR |= 1u;
}


uint8_t uavcan_rx(
    uavcan_frame_id_t *out_message_id,
    size_t *out_length,
    uint8_t *out_message,
    uint8_t fifo
) {
    uint32_t frame_id, rdhr, rdlr;

    /* Check if a message is pending */
    if (fifo == 0u && !(CAN1->RF0R & 3u)) {
        return 0;
    } else if (fifo == 1u && !(CAN1->RF1R & 3u)) {
        return 0;
    } else if (fifo > 1u) {
        return 0;
    }

    /* If so, process it */
    frame_id = CAN1->sFIFOMailBox[fifo].RIR >> 3u;
    *out_length = CAN1->sFIFOMailBox[fifo].RDTR & 0xFu;
    rdhr = CAN1->sFIFOMailBox[fifo].RDHR;
    rdlr = CAN1->sFIFOMailBox[fifo].RDLR;

    /* Release the message from the receive FIFO */
    if (fifo == 0) {
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    } else {
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }

    out_message[0] = ((uint8_t*)&rdlr)[0];
    out_message[1] = ((uint8_t*)&rdlr)[1];
    out_message[2] = ((uint8_t*)&rdlr)[2];
    out_message[3] = ((uint8_t*)&rdlr)[3];

    out_message[4] = ((uint8_t*)&rdhr)[0];
    out_message[5] = ((uint8_t*)&rdhr)[1];
    out_message[6] = ((uint8_t*)&rdhr)[2];
    out_message[7] = ((uint8_t*)&rdhr)[3];

    uavcan_parse_frame_id(out_message_id, frame_id);

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
    while (!g_bootloader_tboot_expired &&
            ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
    last_cyccnt = DWT->CYCCNT;

    /* Start the bit timing loop */
    do {
        last_msr = CAN1->MSR;
        /* Measure the duration of the next transition */
        while (!g_bootloader_tboot_expired &&
            ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
        bit_cycles = DWT->CYCCNT - last_cyccnt;
        last_cyccnt = DWT->CYCCNT;

        /*
        If the number of cycles for the last bit transition was smaller than
        the duration of a bit at the current speed, change to the next higher
        speed.
        */
        if (!g_bootloader_tboot_expired &&
                bit_cycles < current_speed_bit_cycles) {
            /* Increase speed to match the bit timing */
            do {
                current_speed++;
                current_speed_bit_cycles >>= 1u;
            } while (bit_cycles < current_speed_bit_cycles &&
                     current_speed < CAN_1MBAUD);

            status = uavcan_init(current_speed);
            if (status != CAN_OK) {
                return status;
            }

            /* Wait for the next transition */
            last_msr = CAN1->MSR;
            while (!g_bootloader_tboot_expired &&
                   ~(CAN1->MSR ^ last_msr) & CAN_MSR_RX);
            last_cyccnt = DWT->CYCCNT;
        }
    } while (!g_bootloader_tboot_expired && current_speed < CAN_1MBAUD &&
             !(CAN1->RF0R & 3u) && !(CAN1->RF1R & 3u));

    /* Did we time out or get a message? */
    if (g_bootloader_tboot_expired) {
        return CAN_ERROR;
    } else {
        return CAN_OK;
    }
}


size_t uavcan_pack_nodestatus(
    uint8_t *data,
    const uavcan_nodestatus_t *payload
) {
    /* No need to clear the top 4 bits of uptime_sec */
    data[0] = ((uint8_t*)&payload->uptime_sec)[0];
    data[1] = ((uint8_t*)&payload->uptime_sec)[1];
    data[2] = ((uint8_t*)&payload->uptime_sec)[2];
    data[3] = ((uint8_t*)&payload->uptime_sec)[3] | payload->status_code;
    data[4] = ((uint8_t*)&payload->vendor_specific_status_code)[0];
    data[5] = ((uint8_t*)&payload->vendor_specific_status_code)[1];
    return 6u;
}


size_t uavcan_pack_dynamicnodeidallocation(
    uint8_t *data,
    const uavcan_dynamicnodeidallocation_t *payload
) {
    data[0] = ((uint8_t*)&payload->short_unique_id)[0];
    data[1] = ((uint8_t*)&payload->short_unique_id)[1];
    data[2] = ((uint8_t*)&payload->short_unique_id)[2];
    data[3] = ((uint8_t*)&payload->short_unique_id)[3];
    data[4] = ((uint8_t*)&payload->short_unique_id)[4];
    data[5] = ((uint8_t*)&payload->short_unique_id)[5];
    data[6] = ((uint8_t*)&payload->short_unique_id)[6];
    data[7] = (uint8_t)
            (((((uint8_t*)&payload->short_unique_id)[7] & 0x1u) << 7u) |
            (payload->node_id & 0x7Fu));
    return 8u;
}


void uavcan_unpack_dynamicnodeidallocation(
    uavcan_dynamicnodeidallocation_t *payload,
    const uint8_t *data
) {
    ((uint8_t*)&payload->short_unique_id)[0] = data[0];
    ((uint8_t*)&payload->short_unique_id)[1] = data[1];
    ((uint8_t*)&payload->short_unique_id)[2] = data[2];
    ((uint8_t*)&payload->short_unique_id)[3] = data[3];
    ((uint8_t*)&payload->short_unique_id)[4] = data[4];
    ((uint8_t*)&payload->short_unique_id)[5] = data[5];
    ((uint8_t*)&payload->short_unique_id)[6] = data[6];
    ((uint8_t*)&payload->short_unique_id)[7] = data[7] >> 7u;
    payload->node_id = data[7] & 0x7Fu;
}


size_t uavcan_pack_logmessage(
    uint8_t *data,
    const uavcan_logmessage_t *payload
) {
    data[0] = (uint8_t)(((payload->level & 0x7u) << 5u) | 4u);
    data[1] = UAVCAN_LOGMESSAGE_SOURCE_0;
    data[2] = UAVCAN_LOGMESSAGE_SOURCE_1;
    data[3] = UAVCAN_LOGMESSAGE_SOURCE_2;
    data[4] = UAVCAN_LOGMESSAGE_SOURCE_3;
    data[5] = payload->message[0];
    data[6] = payload->message[1];
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


void uavcan_tx_nodestatus(
    uint8_t node_id,
    uint32_t uptime_sec,
    uint8_t status_code
) {
    uavcan_nodestatus_t message;
    uavcan_frame_id_t metadata;
    uint8_t payload[8];
    size_t frame_len;

    metadata.transfer_id = 0;
    metadata.last_frame = 1u;
    metadata.frame_index = 0;
    metadata.source_node_id = node_id;
    metadata.transfer_type = MESSAGE_BROADCAST;
    metadata.data_type_id = UAVCAN_NODESTATUS_DTID;

    message.uptime_sec = uptime_sec;
    message.status_code = status_code;
    frame_len = uavcan_pack_nodestatus(payload, &message);

    uavcan_tx(&metadata, frame_len, payload, 1u);
}


void uavcan_tx_getnodeinfo_response(
    uint8_t node_id,
    uavcan_getnodeinfo_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    /* This sends via mailbox 1 because it's called from SysTick */
    uint32_t i, m;
    uint16_t frame_crc;
    uavcan_frame_id_t message_id;
    uint8_t *message_bytes, payload[8];
    size_t fixed_length, contiguous_length;

    message_bytes = (uint8_t*)response;
    fixed_length = 6u + sizeof(uavcan_softwareversion_t) + 2u + 16u + 1u;

    /*
    Calculate the CRC of the node status, the software version and the
    fixed-length part of the hardware version
    */
    frame_crc = UAVCAN_GETNODEINFO_CRC;
    for (i = 0u; i < fixed_length; i++) {
        frame_crc = crc16_add(frame_crc, message_bytes[i]);
    }
    /* Add the CRC for the variable-length parts */
    for (i = 0u;
            i < response->hardware_version.certificate_of_authenticity_length;
            i++) {
        frame_crc = crc16_add(frame_crc,
            response->hardware_version.certificate_of_authenticity[i]);
    }
    for (i = 0u; i < response->name_length; i++) {
        frame_crc = crc16_add(frame_crc, response->name[i]);
    }
    frame_crc ^= CRC16_OUTPUT_XOR;

    /* Set up the message ID */
    message_id.transfer_id = transfer_id;
    message_id.last_frame = 0u;
    message_id.frame_index = 0u;
    message_id.source_node_id = node_id;
    message_id.transfer_type = SERVICE_RESPONSE;
    message_id.data_type_id = UAVCAN_GETNODEINFO_DTID;

    /*
    Send the fixed-length part of the message plus the COA bytes, since
    they're contiguous
    */
    contiguous_length = fixed_length +
        response->hardware_version.certificate_of_authenticity_length;
    payload[0] = dest_node_id;
    payload[1] = (uint8_t)frame_crc;
    payload[2] = (uint8_t)(frame_crc >> 8u);
    for (i = 0u, m = 3u; i < contiguous_length; i++) {
        payload[m++] = message_bytes[i];
        if (m == 8u) {
            uavcan_tx(&message_id, 8u, payload, 1u);
            message_id.frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /*  Now send the name */
    for (i = 0u; i < response->name_length; i++) {
        payload[m++] = response->name[i];
        if (i == response->name_length - 1u) {
            break;
        } else if (m == 8u) {
            /*
            Don't send the final frame within this loop -- skip if it's about
            to terminate.
            */
            uavcan_tx(&message_id, 8u, payload, 1u);
            message_id.frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last frame */
    message_id.last_frame = 1u;
    uavcan_tx(&message_id, m, payload, 1u);
}


can_error_t uavcan_rx_beginfirmwareupdate_request(
    uint8_t node_id,
    uavcan_beginfirmwareupdate_request_t *request,
    uavcan_frame_id_t *message_id
) {
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_beginfirmwareupdate_request_t) - 1;
    message_id->transfer_id = 0xFFu;
    message_id->source_node_id = 0xFFu;
    message_id->transfer_type = SERVICE_REQUEST;
    message_id->data_type_id = UAVCAN_BEGINFIRMWAREUPDATE_DTID;

    status = uavcan_rx_multiframe_(node_id, message_id, (uint8_t*)request,
                                   &length, UAVCAN_BEGINFIRMWAREUPDATE_CRC,
                                   1000u);

    if (status == CAN_OK && length >= 1u) {
        request->path_length = (uint8_t)(length - 1u);
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


void uavcan_tx_read_request(
    uint8_t node_id,
    const uavcan_read_request_t *request,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    uint32_t i, m;
    uint16_t frame_crc;
    uavcan_frame_id_t message_id;
    const uint8_t *message_bytes;
    uint8_t payload[8];

    message_bytes = (const uint8_t*)request;

    /* Calculate the message CRC */
    frame_crc = UAVCAN_READ_CRC;
    for (i = 0u; i < request->path_length + 4u; i++) {
        frame_crc = crc16_add(frame_crc, message_bytes[i]);
    }
    frame_crc ^= CRC16_OUTPUT_XOR;

    /* Set up the message ID */
    message_id.transfer_id = transfer_id;
    message_id.last_frame = 0u;
    message_id.frame_index = 0u;
    message_id.source_node_id = node_id;
    message_id.transfer_type = SERVICE_REQUEST;
    message_id.data_type_id = UAVCAN_READ_DTID;

    /*
    Send the message -- only prepend CRC if the message will not fit within a
    single frame
    */
    payload[0] = dest_node_id;
    m = 1u;
    if (request->path_length + 4u > 7u) {
        payload[m++] = (uint8_t)frame_crc;
        payload[m++] = (uint8_t)(frame_crc >> 8u);
    }
    for (i = 0u; i < request->path_length + 4u; i++) {
        payload[m++] = message_bytes[i];
        if (i == request->path_length + 4u - 1u) {
            break;
        } else if (m == 8u) {
            uavcan_tx(&message_id, 8u, payload, 0u);
            message_id.frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last (only?) frame */
    message_id.last_frame = 1u;
    uavcan_tx(&message_id, m, payload, 0u);
}


can_error_t uavcan_rx_read_response(
    uint8_t node_id,
    uavcan_read_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t message_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_read_response_t) - 1;
    message_id.transfer_id = transfer_id;
    message_id.source_node_id = dest_node_id;
    message_id.transfer_type = SERVICE_RESPONSE;
    message_id.data_type_id = UAVCAN_READ_DTID;

    status = uavcan_rx_multiframe_(node_id, &message_id, (uint8_t*)response,
                                   &length, UAVCAN_READ_CRC, timeout_ticks);

    if (status == CAN_OK && length >= 2u) {
        response->data_length = (uint8_t)(length - 2u);
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


void uavcan_tx_getinfo_request(
    uint8_t node_id,
    const uavcan_getinfo_request_t *request,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    uint32_t i, m;
    uint16_t frame_crc;
    uavcan_frame_id_t message_id;
    const uint8_t *message_bytes;
    uint8_t payload[8];

    message_bytes = (const uint8_t*)request;

    /* Calculate the message CRC */
    frame_crc = UAVCAN_GETINFO_CRC;
    for (i = 0u; i < request->path_length; i++) {
        frame_crc = crc16_add(frame_crc, message_bytes[i]);
    }
    frame_crc ^= CRC16_OUTPUT_XOR;

    /* Set up the message ID */
    message_id.transfer_id = transfer_id;
    message_id.last_frame = 0u;
    message_id.frame_index = 0u;
    message_id.source_node_id = node_id;
    message_id.transfer_type = SERVICE_REQUEST;
    message_id.data_type_id = UAVCAN_GETINFO_DTID;

    /*
    Send the message -- only prepend CRC if the message will not fit within a
    single frame
    */
    payload[0] = dest_node_id;
    m = 1u;
    if (request->path_length > 7u) {
        payload[m++] = (uint8_t)frame_crc;
        payload[m++] = (uint8_t)(frame_crc >> 8u);
    }

    for (i = 0u; i < request->path_length; i++) {
        payload[m++] = message_bytes[i];
        if (i == request->path_length - 1u) {
            break;
        } else if (m == 8u) {
            uavcan_tx(&message_id, 8u, payload, 0u);
            message_id.frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last (only?) frame */
    message_id.last_frame = 1u;
    uavcan_tx(&message_id, m, payload, 0u);
}


can_error_t uavcan_rx_getinfo_response(
    uint8_t node_id,
    uavcan_getinfo_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t message_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_getinfo_response_t);
    message_id.transfer_id = transfer_id;
    message_id.source_node_id = dest_node_id;
    message_id.transfer_type = SERVICE_RESPONSE;
    message_id.data_type_id = UAVCAN_GETINFO_DTID;

    status = uavcan_rx_multiframe_(node_id, &message_id, (uint8_t*)response,
                                   &length, UAVCAN_GETINFO_CRC,
                                   timeout_ticks);

    if (status == CAN_OK && length == sizeof(uavcan_getinfo_response_t)) {
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


static can_error_t uavcan_rx_multiframe_(
    uint8_t node_id,
    uavcan_frame_id_t *message_id,
    uint8_t *message,
    size_t *message_length,
    uint16_t initial_crc,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t rx_id;
    size_t rx_length, m, i;
    uint32_t start_ticks;
    uint16_t message_crc, calculated_crc;
    uint8_t payload[8];
    uint8_t got_frame, num_frames;

    start_ticks = g_bootloader_uptime;

    num_frames = 0u;
    rx_id.last_frame = 0u;
    message_crc = 0u;
    i = 0;

    do {
        got_frame = uavcan_rx(&rx_id, &rx_length, payload, 0u);
        if (got_frame &&
                (message_id->source_node_id == 0xFFu ||
                    rx_id.source_node_id == message_id->source_node_id) &&
                (message_id->transfer_id == 0xFFu ||
                    rx_id.transfer_id == message_id->transfer_id) &&
                rx_id.transfer_type == message_id->transfer_type &&
                rx_id.data_type_id == message_id->data_type_id &&
                rx_id.frame_index == num_frames &&
                payload[0] == node_id) {
            /*
            Get the CRC if this is the first frame of a multi-frame transfer.

            Also increase the timeout to UAVCAN_SERVICE_TIMEOUT_TICKS.
            */
            if (num_frames == 0u && !rx_id.last_frame) {
                timeout_ticks = UAVCAN_SERVICE_TIMEOUT_TICKS;
                message_crc = (uint16_t)(payload[1] | (payload[2] << 8u));
                m = 3u;
            } else {
                m = 1u;
            }

            /* Copy message bytes to the response */
            for (; m < rx_length && i < *message_length; m++, i++) {
                message[i] = payload[m];
            }
            num_frames++;

            if (message_id->transfer_id == 0xFFu) {
                message_id->transfer_id = rx_id.transfer_id;
            }
            if (message_id->source_node_id == 0xFFu) {
                message_id->source_node_id = rx_id.source_node_id;
            }

            if (rx_id.last_frame) {
                break;
            }
        }
    } while (g_bootloader_uptime - start_ticks < timeout_ticks);

    if (!rx_id.last_frame) {
        return CAN_ERROR;
    } else {
        /* Validate CRC */
        calculated_crc = initial_crc;
        for (m = 0u; m < i; m++) {
            calculated_crc = crc16_add(calculated_crc, message[m]);
        }
        calculated_crc ^= CRC16_OUTPUT_XOR;

        *message_length = i;

        if (num_frames == 1u || message_crc == calculated_crc) {
            return CAN_OK;
        } else {
            return CAN_ERROR;
        }
    }
}
