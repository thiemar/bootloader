#pragma once


typedef enum {
    CAN_OK = 0,
    CAN_ERROR
} can_error_t;


typedef enum {
    CAN_125KBAUD = 0,
    CAN_250KBAUD = 1,
    CAN_500KBAUD = 2,
    CAN_1MBAUD = 3
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
can_error_t can_autobaud(void);
