#pragma once


#include "uavcan.h"


#define OPT_TBOOT_MS 2000u
#define OPT_CPU_FREQ_HZ 72000000u
#define OPT_WAIT_FOR_GETNODEINFO
/* #define OPT_WAIT_FOR_GETNODEINFO_FLAG */
#define OPT_ENABLE_WD
#define OPT_MIN_APP_PROGRESS 0u
#define OPT_RESTART_TIMEOUT_MS 20000u

#define OPT_APPLICATION_IMAGE_OFFSET 8192u
#define OPT_APPLICATION_IMAGE_LENGTH 49152u

#define FLASH_PAGE_SIZE 2048u
#define FLASH_SIZE 65536u

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
uint8_t board_get_wait_for_getnodeinfo_flag(void);
