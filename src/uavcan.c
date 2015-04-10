#include <stdint.h>
#include <stdlib.h>
#include "bootloader_config.h"
#include "uavcan.h"
#include "can.h"
#include "crc.h"


extern volatile uint32_t g_bootloader_uptime;


static void uavcan_tx_multiframe_(
    uavcan_frame_id_t *frame_id,
    uint8_t dest_node_id,
    size_t message_length,
    const uint8_t *message,
    uint16_t initial_crc
);
static can_error_t uavcan_rx_multiframe_(
    uint8_t node_id,
    uavcan_frame_id_t *frame_id,
    size_t *message_length,
    uint8_t *message,
    uint16_t initial_crc,
    uint32_t timeout_cycles
);


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
    size_t i;
    for (i = 0u; i < 7u; i++) {
        data[i] = ((uint8_t*)&payload->short_unique_id)[i];
    }
    data[7] = (uint8_t)
            (((((uint8_t*)&payload->short_unique_id)[7] & 0x1u) << 7u) |
            (payload->node_id & 0x7Fu));
    return 8u;
}


void uavcan_unpack_dynamicnodeidallocation(
    uavcan_dynamicnodeidallocation_t *payload,
    const uint8_t *data
) {
    size_t i;
    for (i = 0u; i < 7u; i++) {
        ((uint8_t*)&payload->short_unique_id)[i] = data[i];
    }
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


uint32_t uavcan_make_message_id(const uavcan_frame_id_t *frame_id) {
    return (frame_id->transfer_id & 0x7u) |
           ((frame_id->last_frame ? 1u : 0u) << 3u) |
           ((frame_id->frame_index & 0x3Fu) << 4u) |
           ((frame_id->source_node_id & 0x7Fu) << 10u) |
           ((frame_id->transfer_type & 0x3u) << 17u) |
           ((frame_id->data_type_id & 0x3FFu) << 19u);
}


void uavcan_parse_message_id(
    uavcan_frame_id_t *frame_id,
    uint32_t message_id
) {
    frame_id->transfer_id = message_id & 0x7u;
    frame_id->last_frame = (message_id & 0x8u) ? 1u : 0u;
    frame_id->frame_index = (message_id >> 4u) & 0x3Fu;
    frame_id->source_node_id = (message_id >> 10u) & 0x7Fu;
    frame_id->transfer_type = (message_id >> 17u) & 0x3u;
    frame_id->data_type_id = (message_id >> 19u) & 0x3FFu;
}


void uavcan_tx_nodestatus(
    uint8_t node_id,
    uint32_t uptime_sec,
    uint8_t status_code
) {
    uavcan_nodestatus_t message;
    uavcan_frame_id_t frame_id;
    uint8_t payload[8];
    size_t frame_len;

    frame_id.transfer_id = 0;
    frame_id.last_frame = 1u;
    frame_id.frame_index = 0;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = MESSAGE_BROADCAST;
    frame_id.data_type_id = UAVCAN_NODESTATUS_DTID;

    message.uptime_sec = uptime_sec;
    message.status_code = status_code;
    message.vendor_specific_status_code = 0u;
    frame_len = uavcan_pack_nodestatus(payload, &message);

    can_tx(uavcan_make_message_id(&frame_id), frame_len, payload, 1u);
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
    uavcan_frame_id_t frame_id;
    uint8_t *message_bytes, payload[8];
    size_t fixed_length, contiguous_length;

    message_bytes = (uint8_t*)response;
    fixed_length = 6u + sizeof(uavcan_softwareversion_t) + 2u + 16u + 1u;

    /*
    Calculate the CRC of the node status, the software version and the
    fixed-length part of the hardware version
    */
    frame_crc = crc16_signature(UAVCAN_GETNODEINFO_CRC, fixed_length,
                                message_bytes);
    frame_crc ^= CRC16_OUTPUT_XOR;
    /* Add the CRC for the variable-length parts */
    frame_crc = crc16_signature(
        frame_crc,
        response->hardware_version.certificate_of_authenticity_length,
        response->hardware_version.certificate_of_authenticity);
    frame_crc ^= CRC16_OUTPUT_XOR;
    frame_crc = crc16_signature(frame_crc, response->name_length,
                                response->name);

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.last_frame = 0u;
    frame_id.frame_index = 0u;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_GETNODEINFO_DTID;

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
            can_tx(uavcan_make_message_id(&frame_id), 8u, payload, 1u);
            frame_id.frame_index++;
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
            can_tx(uavcan_make_message_id(&frame_id), 8u, payload, 1u);
            frame_id.frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last frame */
    frame_id.last_frame = 1u;
    can_tx(uavcan_make_message_id(&frame_id), m, payload, 1u);
}


can_error_t uavcan_rx_beginfirmwareupdate_request(
    uint8_t node_id,
    uavcan_beginfirmwareupdate_request_t *request,
    uavcan_frame_id_t *frame_id
) {
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_beginfirmwareupdate_request_t) - 1;
    frame_id->transfer_id = 0xFFu;
    frame_id->source_node_id = 0xFFu;
    frame_id->transfer_type = SERVICE_REQUEST;
    frame_id->data_type_id = UAVCAN_BEGINFIRMWAREUPDATE_DTID;

    status = uavcan_rx_multiframe_(node_id, frame_id, &length,
                                   (uint8_t*)request,
                                   UAVCAN_BEGINFIRMWAREUPDATE_CRC, 100u);

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
    uavcan_frame_id_t frame_id;

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_REQUEST;
    frame_id.data_type_id = UAVCAN_READ_DTID;

    uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length + 4u,
                          (const uint8_t*)request, UAVCAN_READ_CRC);
}


can_error_t uavcan_rx_read_response(
    uint8_t node_id,
    uavcan_read_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t frame_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_read_response_t) - 1;
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = dest_node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_READ_DTID;

    status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
                                   (uint8_t*)response, UAVCAN_READ_CRC,
                                   timeout_ticks);

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
    uavcan_frame_id_t frame_id;

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_REQUEST;
    frame_id.data_type_id = UAVCAN_GETINFO_DTID;

    uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length,
                          (const uint8_t*)request, UAVCAN_GETINFO_CRC);
}


can_error_t uavcan_rx_getinfo_response(
    uint8_t node_id,
    uavcan_getinfo_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t frame_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_getinfo_response_t);
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = dest_node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_GETINFO_DTID;

    status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
                                   (uint8_t*)response, UAVCAN_GETINFO_CRC,
                                   timeout_ticks);

    if (status == CAN_OK && length == sizeof(uavcan_getinfo_response_t)) {
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


static void uavcan_tx_multiframe_(
    uavcan_frame_id_t *frame_id,
    uint8_t dest_node_id,
    size_t message_length,
    const uint8_t *message,
    uint16_t initial_crc
) {
    uint32_t i, m;
    uint16_t frame_crc;
    uint8_t payload[8];

    /* Calculate the message CRC */
    frame_crc = crc16_signature(initial_crc, message_length, message);

    /* Ensure message ID has the frame details zeroed */
    frame_id->last_frame = 0u;
    frame_id->frame_index = 0u;

    /*
    Send the message -- only prepend CRC if the message will not fit within a
    single frame
    */
    payload[0] = dest_node_id;
    m = 1u;
    if (message_length > 7u) {
        payload[m++] = (uint8_t)frame_crc;
        payload[m++] = (uint8_t)(frame_crc >> 8u);
    }
    for (i = 0u; i < message_length; i++) {
        payload[m++] = message[i];
        if (i == message_length - 1u) {
            break;
        } else if (m == 8u) {
            can_tx(uavcan_make_message_id(frame_id), 8u, payload, 0u);
            frame_id->frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last (only?) frame */
    frame_id->last_frame = 1u;
    can_tx(uavcan_make_message_id(frame_id), m, payload, 0u);
}


static can_error_t uavcan_rx_multiframe_(
    uint8_t node_id,
    uavcan_frame_id_t *frame_id,
    size_t *message_length,
    uint8_t *message,
    uint16_t initial_crc,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t rx_id;
    size_t rx_length, m, i;
    uint32_t start_ticks, rx_message_id;
    uint16_t message_crc, calculated_crc;
    uint8_t payload[8];
    uint8_t got_frame, num_frames;

    start_ticks = g_bootloader_uptime;

    num_frames = 0u;
    rx_id.last_frame = 0u;
    message_crc = 0u;
    i = 0;

    do {
        rx_message_id = 0u;
        rx_length = 0u;
        got_frame = can_rx(&rx_message_id, &rx_length, payload, 0u);
        uavcan_parse_message_id(&rx_id, rx_message_id);
        if (got_frame &&
                (frame_id->source_node_id == 0xFFu ||
                    rx_id.source_node_id == frame_id->source_node_id) &&
                (frame_id->transfer_id == 0xFFu ||
                    (rx_id.transfer_id & 7u) ==
                    (frame_id->transfer_id & 7u)) &&
                rx_id.transfer_type == frame_id->transfer_type &&
                rx_id.data_type_id == frame_id->data_type_id &&
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

            if (frame_id->transfer_id == 0xFFu) {
                frame_id->transfer_id = rx_id.transfer_id;
            }
            if (frame_id->source_node_id == 0xFFu) {
                frame_id->source_node_id = rx_id.source_node_id;
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
        calculated_crc = crc16_signature(initial_crc, i, message);

        *message_length = i;

        if (num_frames == 1u || message_crc == calculated_crc) {
            return CAN_OK;
        } else {
            return CAN_ERROR;
        }
    }
}
