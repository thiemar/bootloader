#pragma once


#include "uavcan.h"


/*
Application firmware descriptor. This goes at the start of the application,
and is 128 bytes long since that's the alignment requirement of the
Cortex-M3/-M4 NVIC vector offset field.

(Cortex-M0 doesn't allow the vector offset to be modified, so on that chip
we'll need to have the bootloader do its own exception remapping.)

The descriptor_magic field must be set to some well-known value, and is the
last word to be written during the firmware upgrade process so if it's
incorrect, the firmware image is not valid.

The CRC is calculated over all fields as well as the firmware itself,
although for the purposes of CRC calculation the image_crc field is set to 0.
The length is the length of the entire image, including the 128-byte
descriptor.
*/
typedef struct {
    uint32_t descriptor_magic;
    uint32_t image_size;
    uint64_t image_crc;
    uint32_t vcs_commit;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t reserved[106];
} __attribute__((packed)) fw_image_descriptor_t;


#define OPT_TBOOT_MS 2000u
#define OPT_CPU_FREQ_HZ 72000000u
#define OPT_WAIT_FOR_GETNODEINFO
#define OPT_ENABLE_WD
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO
#define OPT_RESTART_TIMEOUT_MS 20000u

#define OPT_APPLICATION_IMAGE_OFFSET 8192u
#define OPT_APPLICATION_IMAGE_LENGTH 49152u
#define OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC 0xB007C0DEu

#define PORT_CAN_SILENT GPIOB
#define PIN_CAN_SILENT_INDEX 6

#define PORT_CAN_RX GPIOA
#define PIN_CAN_RX_INDEX 11

#define PORT_CAN_TX GPIOA
#define PIN_CAN_TX_INDEX 12

/* TODO: OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO pin settings*/

#define FLASH_PAGE_SIZE 2048u


#define UAVCAN_SERVICE_RETRIES 3u
#define UAVCAN_SERVICE_TIMEOUT_TICKS 100u
#define UAVCAN_NODESTATUS_INTERVAL_TICKS 50u


uint8_t board_get_product_name(uint8_t *product_name);
void board_get_hardware_version(uavcan_hardwareversion_t *hw_version);
void board_initialize(void);
void board_indicate_reset(void);
void board_indicate_autobaud_start(void);
void board_indicate_autobaud_end(void);
void board_indicate_allocation_start(void);
void board_indicate_allocation_end(void);
void board_indicate_fw_update_start(void);
void board_indicate_fw_update_erase_fail(void);
void board_indicate_fw_update_invalid_response(void);
void board_indicate_fw_update_timeout(void);
void board_indicate_fw_update_invalid_crc(void);
