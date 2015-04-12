#pragma once


typedef enum {
    CAN_OK = 0u,
    CAN_ERROR
} can_error_t;


typedef enum {
    CAN_UNKNOWN = 0u,
    CAN_125KBAUD = 1u,
    CAN_250KBAUD = 2u,
    CAN_500KBAUD = 3u,
    CAN_1MBAUD = 4u
} can_speed_t;


void can_tx(
    uint32_t message_id,
    size_t length,
    const uint8_t *message,
    uint8_t mailbox
);
uint8_t can_rx(
    uint32_t *message_id,
    size_t *out_length,
    uint8_t *out_message,
    uint8_t fifo
);
can_error_t can_init(can_speed_t speed, uint8_t silent);
can_speed_t can_autobaud(void);
