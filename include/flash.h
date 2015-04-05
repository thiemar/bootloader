#pragma once

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR
} flash_error_t;

flash_error_t flash_wait(void);
flash_error_t flash_erase(void);
flash_error_t flash_write_data(
    uint32_t flash_address,
    size_t length,
    const uint8_t* data
);
uint64_t flash_crc(
    uint32_t flash_address,
    size_t length
);
