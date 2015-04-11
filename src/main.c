#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "cmsis_device.h"
#include "bootloader_config.h"
#include "flash.h"
#include "uavcan.h"
#include "crc.h"
#include "common.h"


/* FIXME */
extern void SystemInit(void);


void SysTick_Handler(void);
void bootloader_main(void);
void bootloader_gpio_init(void);
void bootloader_reset(void);
uint64_t bootloader_get_short_unique_id(void);
void bootloader_send_log_message(
    uint8_t node_id,
    uint8_t level,
    uint8_t stage,
    uint8_t status
);
uint8_t bootloader_timeout(uint32_t start_ticks, uint32_t ticks);
uint8_t bootloader_get_dynamic_node_id(void);
void bootloader_autobaud_and_get_dynamic_node_id(void);
void bootloader_poll_getnodeinfo(
    uint8_t node_id,
    uint32_t uptime,
    uint8_t status
);
void bootloader_wait_for_beginfirmwareupdate(
    uint8_t *fw_path,
    uint8_t *fw_path_length,
    uint8_t *fw_source_node_id
);
void bootloader_file_getinfo(
    size_t *fw_image_size,
    uint64_t *fw_image_crc,
    const uint8_t *fw_path,
    uint8_t fw_path_length,
    uint8_t fw_source_node_id
);
flash_error_t bootloader_file_read_and_program(
    uint8_t fw_source_node_id,
    uint8_t fw_path_length,
    const uint8_t *fw_path,
    size_t fw_image_size,
    uint32_t *fw_word0
);
uint8_t bootloader_is_app_valid(uint32_t first_word);
void bootloader_find_descriptor(void);
void application_run(void);


volatile uint8_t g_bootloader_node_id;
volatile uint8_t g_bootloader_status_code;
volatile uint8_t g_bootloader_got_node_info;
volatile uint8_t g_bootloader_app_valid;
volatile uint32_t g_bootloader_uptime;
volatile uint32_t g_bootloader_bus_speed;

volatile uint32_t g_bootloader_tboot_deadline;
volatile uint8_t g_bootloader_tboot_expired;
volatile uint8_t g_bootloader_tboot_enable;

volatile app_descriptor_t *g_fw_image_descriptor;
volatile uint32_t *g_fw_image;

/*
SysTick task executes at 100 Hz and performs the following actions:
* Counts uptime seconds in g_bootloader_uptime;
* Sets g_bootloader_tboot_timeout to 1 once Tboot has elapsed;
* Sends uavcan.protocol.NodeStatus at 100 Hz
* Replies to uavcan.protocol.GetNodeInfo requests

UAVCANBootloader_v0.3 #21.2: 550.NodeStatus.uavcan(uptime=0, STATUS_INITIALIZING)
UAVCANBootloader_v0.3 #24: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
UAVCANBootloader_v0.3 #28.5: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
UAVCANBootloader_v0.3 #49: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
*/
void SysTick_Handler(void) {
    uint8_t node_id, status;
    uint32_t uptime;

    /* Update uptime and check for Tboot expiry */
    uptime = ++g_bootloader_uptime;
    if (uptime > g_bootloader_tboot_deadline && g_bootloader_tboot_enable) {
        g_bootloader_tboot_expired = 1u;
    }

    node_id = g_bootloader_node_id;
    if (node_id) {
        status = g_bootloader_status_code;

        /* Handle GetNodeInfo replies regardless of application state */
        bootloader_poll_getnodeinfo(node_id, uptime / 100u, status);

        /* Send uavcan.protocol.NodeStatus every 50 ticks */
        if (uptime % UAVCAN_NODESTATUS_INTERVAL_TICKS == 0) {
            uavcan_tx_nodestatus(node_id, uptime / 100u, status);
        }
    }
}


void
__attribute__((noreturn))
bootloader_main(void) {
    uint64_t fw_image_crc;
    size_t fw_image_size;
    uint32_t restart_ticks, fw_word0;
    uint8_t fw_path[200], fw_path_length, fw_source_node_id;
    uint8_t app_bl_request, wait_for_getnodeinfo, error_log_stage,
            common_valid;
    flash_error_t status;
    bootloader_app_common_t common;

    /* UAVCANBootloader_v0.3 #1: Reset */
    error_log_stage = UAVCAN_LOGMESSAGE_STAGE_INIT;

    /* UAVCANBootloader_v0.3 #2: Clock, IO Init */
    board_initialize();

    SysTick->LOAD = OPT_CPU_FREQ_HZ / 100u - 1u;
    SysTick->VAL = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* TODO UAVCANBootloader_v0.3 #4: EnableWatchDog */
#ifdef OPT_ENABLE_WD
    /* ... */
#endif

    /* UAVCANBootloader_v0.3 #6: INDICATE_RESET */
    board_indicate_reset();

    /* UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) && ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
    g_fw_image =
        (volatile uint32_t*)(FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET);

    /* Find the image descriptor and validate the image */
    g_bootloader_app_valid = bootloader_is_app_valid(g_fw_image[0]);

    /* UAVCANBootloader_v0.3 #8: InitNodeInfo(AppValid) */
    g_bootloader_got_node_info = 0u;
    g_bootloader_node_id = 0u;

    /* UAVCANBootloader_v0.3 #9: bool AppBLRequest = Validate(BootLoaderAppCommon, AP) */
    common_valid = common_is_valid();
    if (common_valid) {
        /* UAVCANBootloader_v0.3 #10: Init(BootLoaderAppCommon, BL, AppBLRequest) */
        common_read(&common);
        /*
        Invalidate the common structure as we don't want to keep reading the
        same parameters on future resets if they might not be working.
        */
        common_invalidate();

        app_bl_request =
            common.signature == BOOTLOADER_COMMON_APP_SIGNATURE &&
            common.bus_speed && common.node_id && !common.app_progress;

        /*
        Consider app invalid if the signature is our own (meaning the app
        didn't change the filters) or if less than the minimum required
        progress has been made.
        */
        g_bootloader_app_valid =
            g_bootloader_app_valid &&
            common.app_progress >= OPT_MIN_APP_PROGRESS &&
            common.signature != BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE;
    } else {
        app_bl_request = 0u;
    }

    /* UAVCANBootloader_v0.3 #11: bool WaitForGetNodeInfo = -DOPT_WAIT_FOR_GET_NODE_INFO */
#if defined(OPT_WAIT_FOR_GETNODEINFO)
    wait_for_getnodeinfo = 1u;
#elif defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
    /* UAVCANBootloader_v0.3 #12: [ -DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO] : WaitForGetNodeInfo = IORead(-DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO) */
    wait_for_getnodeinfo = board_get_wait_for_getnodeinfo_flag();
#else
    wait_for_getnodeinfo = 0u;
#endif

    /* UAVCANBootloader_v0.3 #13: [ !WaitForGetNodeInfo && AppValid]: StartTboot([-DOPT_TBOOT) */
    g_bootloader_tboot_enable = 0u;
    g_bootloader_tboot_expired = 0u;
    g_bootloader_tboot_deadline = 0u;
    g_bootloader_status_code = UAVCAN_NODESTATUS_STATUS_INITIALIZING;
    g_bootloader_uptime = 0u;
    g_bootloader_tboot_deadline = OPT_TBOOT_MS / 10u;
    if (!wait_for_getnodeinfo && !app_bl_request && g_bootloader_app_valid) {
        g_bootloader_tboot_enable = 1u;
    }

    /* Start SysTick interrupt for Tboot and NodeStatus messages */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    /* UAVCANBootloader_v0.3 #14: [AppBLRequest]:RestartFromApp */
    if (!app_bl_request) {
        /*
        Allow an extra Tboot seconds for node ID allocation (won't do anything
        if Tboot is not currently enabled)
        */
        g_bootloader_tboot_deadline += OPT_TBOOT_MS / 10u;

        bootloader_autobaud_and_get_dynamic_node_id();
        if (g_bootloader_tboot_expired) {
            goto boot;
        } else {
            /* UAVCANBootloader_v0.3 #18.2: [!AppBLRequest]:Update(BootLoaderAppCommon, BL) */
            /* UAVCANBootloader_v0.3 #19.1: SetNodeID(newNodeID), Update(BootLoaderAppCommon, BL) */
            common.node_id = g_bootloader_node_id;
            common.bus_speed =
                g_bootloader_bus_speed == CAN_1MBAUD   ? 1000000u :
                g_bootloader_bus_speed == CAN_500KBAUD ?  500000u :
                g_bootloader_bus_speed == CAN_250KBAUD ?  250000u : 125000u;
        }

        /* UAVCANBootloader_v0.3 #21.1: initSeconds(0) */
        g_bootloader_tboot_deadline -= g_bootloader_uptime;
        g_bootloader_uptime = 0u;
    } else {
        /* UAVCANBootloader_v0.3 #21: RestartFromApp */
        g_bootloader_node_id = (uint8_t)common.node_id;
        g_bootloader_bus_speed = common.bus_speed;
        /* Set the bus speed, defaulting to 125 Kbaud if the value is non-zero but invalid */
        can_init(common.bus_speed == 1000000u ? CAN_1MBAUD :
                 common.bus_speed ==  500000u ? CAN_500KBAUD :
                 common.bus_speed ==  250000u ? CAN_250KBAUD : CAN_125KBAUD, 0u);
    }

    /* UAVCANBootloader_v0.3 #21.2.1: Req551.GetNodeInfo.uavcan() */
    do {
        /* UAVCANBootloader_v0.3 #21.2.1.4: [Timeout(Tboot)]: jump_to_app */
        if (g_bootloader_tboot_expired) {
            goto boot;
        }
    } while (!g_bootloader_got_node_info);

    /* UAVCANBootloader_v0.3 #21.2.1.1: [(AppBLRequest || WaitForGetNodeInfo) && AppValid)]: StartTboot([-DOPT_TBOOT) */
    if (!g_bootloader_tboot_enable && g_bootloader_app_valid) {
        g_bootloader_tboot_enable = 1u;
        g_bootloader_tboot_deadline = g_bootloader_uptime +
                                      OPT_TBOOT_MS / 10u;
    }

    /* UAVCANBootloader_v0.3 #22: Req580.BeginFirmwareUpdate.uavcan */
    do {
        bootloader_wait_for_beginfirmwareupdate(fw_path,
                                                &fw_path_length,
                                                &fw_source_node_id);
        if (g_bootloader_tboot_expired) {
            goto boot;
        }
    } while (!fw_source_node_id);

    /*
    From here on, we don't care about Tboot -- the firmware update process
    has started.
    */

    /*
    Don't need in this implementation since BeginFirmwareUpdate is ignored
    when a firmware update is in progress:
    UAVCANBootloader_v0.3 #22.1: BootLoadInProcess = true
    */

    /* UAVCANBootloader_v0.3 #23: INDICATE_FW_UPDATE_START */
    board_indicate_fw_update_start();

    /* UAVCANBootloader_v0.3 #26: 585.GetInfo.uavcan(path) */
    bootloader_file_getinfo(&fw_image_size, &fw_image_crc,
                            fw_path, fw_path_length, fw_source_node_id);
    if (fw_image_size < sizeof(app_descriptor_t) || !fw_image_crc) {
        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_GET_INFO;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #28.6: 1023.LogMessage.uavcan("Erase") */
    bootloader_send_log_message(g_bootloader_node_id,
                                UAVCAN_LOGMESSAGE_LEVEL_INFO,
                                UAVCAN_LOGMESSAGE_STAGE_ERASE,
                                UAVCAN_LOGMESSAGE_RESULT_START);

    /* UAVCANBootloader_v0.3 #28.7: EraseFlash() */
    status = flash_erase();
    if (status != FLASH_OK) {
        /* UAVCANBootloader_v0.3 #28.8: [Erase Failed]:INDICATE_FW_UPDATE_ERASE_FAIL */
        board_indicate_fw_update_erase_fail();

        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_ERASE;
        goto failure;
    }

    status = bootloader_file_read_and_program(fw_source_node_id,
                                              fw_path_length, fw_path,
                                              fw_image_size, &fw_word0);
    if (status != FLASH_OK) {
        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_PROGRAM;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #41: CalulateCRC(file_info) */
    if (!bootloader_is_app_valid(fw_word0)) {
        g_bootloader_app_valid = 0u;

        /* UAVCANBootloader_v0.3 #43: [crc Fail]:INDICATE_FW_UPDATE_INVALID_CRC */
        board_indicate_fw_update_invalid_crc();

        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_VALIDATE;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #46: [crc == fw_crc]:write word0 */
    status = flash_write_word((uint32_t)g_fw_image, (uint8_t*)&fw_word0);
    if (status != FLASH_OK) {
        /* Not specifically listed in UAVCANBootloader_v0.3: 1023.LogMessage.uavcan */
        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_FINALIZE;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #47: ValidateBootLoaderAppCommon() */
    common_write(&common);

    /* Send a completion log message */
    bootloader_send_log_message(g_bootloader_node_id,
                                UAVCAN_LOGMESSAGE_LEVEL_INFO,
                                UAVCAN_LOGMESSAGE_STAGE_FINALIZE,
                                UAVCAN_LOGMESSAGE_RESULT_OK);

    /* TODO UAVCANBootloader_v0.3 #48: KickTheDog() */

boot:
    /* UAVCANBootloader_v0.3 #50: jump_to_app */
    application_run();

    /* We shouldn't ever fall through to the below */

failure:
    /* UAVCANBootloader_v0.3 #28.2: [!validateFileInfo(file_info)]:1023.LogMessage.uavcan (errorcode) */
    /* UAVCANBootloader_v0.3 #28.9: [Erase Fail]:1023.LogMessage.uavcan */
    /* UAVCANBootloader_v0.3 #31: [Program Fail]::1023.LogMessage.uavcan */
    /* UAVCANBootloader_v0.3 #38: [(retries == 0 && timeout)]:1023.LogMessage.uavcan */
    /* UAVCANBootloader_v0.3 #42: [crc Faill]:1023.LogMessage.uavcan */
    bootloader_send_log_message(g_bootloader_node_id,
                                UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                error_log_stage,
                                UAVCAN_LOGMESSAGE_RESULT_FAIL);

    /* UAVCANBootloader_v0.3 #28.3: [!validateFileInfo(file_info)]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #28.10: [Erase Fail]:550.NodeStatus.uavcan(uptime=t,STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #32: [Program Fail]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #39: [(retries == 0 && timeout)]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #44: [crc Fail]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    g_bootloader_status_code = UAVCAN_NODESTATUS_STATUS_CRITICAL;

    /* UAVCANBootloader_v0.3 #28.1: [Retries==0]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
    /* UAVCANBootloader_v0.3 #40: [Retries==0]:InvalidateBootLoaderAppCommon(), RestartWithDelay(20,000ms) */
    /* UAVCANBootloader_v0.3 #45: [crc Fail]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
    restart_ticks = g_bootloader_uptime;
    while (!bootloader_timeout(restart_ticks, OPT_RESTART_TIMEOUT_MS / 10u));

    bootloader_reset();
}


void __attribute__((noreturn)) bootloader_reset(void) {
    __DSB();
    SCB->AIRCR = ((0x5FAu << SCB_AIRCR_VECTKEY_Pos) |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();

    while (1);
}


void bootloader_autobaud_and_get_dynamic_node_id(void) {
    /* UAVCANBootloader_v0.3 #15: [!AppBLRequest]:INDICATE_AUTOBAUDING */
    board_indicate_autobaud_start();

    /* UAVCANBootloader_v0.3 #16: [!AppBLRequest]:Auto Baud */
    /* UAVCANBootloader_v0.3 #17: [!AppBLRequest]:detect bitrate */
    g_bootloader_bus_speed = can_autobaud(); /* does UAVCANBootloader_v0.3 #18.1 */

    /* UAVCANBootloader_v0.3 #18.4: [Timeout(Tboot)]: jump_to_app (node_id is 0 => App need to Allocate or use Static NodeID */
    if (g_bootloader_tboot_expired) {
        return;
    }

    /* UAVCANBootloader_v0.3 #18.3: [!AppBLRequest]:INDICATE_AUTOBAUDED */
    board_indicate_autobaud_end();

    /* UAVCANBootloader_v0.3 #18.5: [!AppBLRequest]:INDICATE_ALLOCATION */
    board_indicate_allocation_start();

    g_bootloader_node_id = bootloader_get_dynamic_node_id();
    /*
    Not needed in this implementation as Tboot timeout checked on return:
    UAVCANBootloader_v0.3 #20: [Timeout(Tboot)]: jump_to_app (node_id is 0 => App need to Allocate or use Static
    */

    /* UAVCANBootloader_v0.3 #19.2: INDICATE_ALLOCATED */
    board_indicate_allocation_end();
}


uint8_t bootloader_get_dynamic_node_id(void) {
    uavcan_hardwareversion_t hw_version;
    uavcan_frame_id_t rx_frame_id;
    uint32_t request_deadline_ticks, rx_message_id;
    uint8_t rx_payload[8], node_id, got_allocation, server_node_id,
            server_transfer_id, server_allocated_node_id, server_frame_index,
            unique_id_matched, transfer_id;
    size_t rx_len, i, offset;
    uint16_t rx_crc, expected_crc;

    /* UAVCANBootloader_v0.3 #18.6: GetUUID() */
    board_get_hardware_version(&hw_version);

    node_id = 0u;
    transfer_id = 0u;
    got_allocation = 0u;
    unique_id_matched = 0u;
    server_node_id = 0u;
    server_transfer_id = 0u;
    server_frame_index = 0u;
    server_allocated_node_id = 0u;
    rx_crc = 0u;

    /*
    Rule A: on initialization, the client subscribes to
    uavcan.protocol.dynamic_node_id.Allocation and starts a Request Timer with
    interval of Trequestinterval = 1 second
    */
    request_deadline_ticks = g_bootloader_uptime + 100u; /* TODO setting? */

    do {
        /*
        Rule B. On expiration of Request Timer:
        1) Request Timer restarts.
        2) The client broadcasts a first-stage Allocation request message,
           where the fields are assigned following values:
                node_id                 - preferred node ID, or zero if the
                                          client doesn't have any preference
                first_part_of_unique_id - true
                unique_id               - first 7 bytes of unique ID
        */
        if (g_bootloader_uptime >= request_deadline_ticks) {
            uavcan_tx_allocation_message(node_id, 16u, hw_version.unique_id,
                                         0u, transfer_id++);
            request_deadline_ticks = g_bootloader_uptime + 100u;
        }


        got_allocation = can_rx(&rx_message_id, &rx_len, rx_payload, 0u);
        if (!got_allocation) {
            continue;
        }

        uavcan_parse_message_id(&rx_frame_id, rx_message_id);
        if (rx_frame_id.data_type_id != UAVCAN_DYNAMICNODEIDALLOCATION_DTID) {
            continue;
        }

        /*
        Rule C. On any Allocation message, even if other rules also match:
        1) Request Timer restarts.
        */
        request_deadline_ticks = g_bootloader_uptime + 100u;

        /*
        Skip this message if it's anonymous (from another client), or if it's
        from a different server to the one we're listening to at the moment
        */
        if (!rx_frame_id.source_node_id || (server_node_id &&
                    rx_frame_id.source_node_id != server_node_id)) {
            continue;
        } else if (!server_node_id ||
                    rx_frame_id.transfer_id != server_transfer_id ||
                    rx_frame_id.frame_index == 0) {
            /* First (only?) frame of the transfer */
            if (rx_frame_id.last_frame) {
                offset = 1u;
            } else {
                rx_crc = (uint16_t)(rx_payload[0] | (rx_payload[1] << 8u));
                server_allocated_node_id = rx_payload[2];
                offset = 3u;
            }
            server_node_id = rx_frame_id.source_node_id;
            server_transfer_id = rx_frame_id.transfer_id;
            unique_id_matched = 0u;
            server_frame_index = 1u;
        } else if (rx_frame_id.frame_index != server_frame_index) {
            /* Abort if the frame index is wrong */
            server_node_id = 0u;
            continue;
        } else {
            offset = 0u;
            server_frame_index++;
        }

        /*
        Rule D. On an Allocation message WHERE (source node ID is
        non-anonymous) AND (client's unique ID starts with the bytes available
        in the field unique_id) AND (unique_id is less than 16 bytes long):
        1) The client broadcasts a second-stage Allocation request message,
           where the fields are assigned following values:
                node_id                 - same value as in the first-stage
                first_part_of_unique_id - false
                unique_id               - at most 7 bytes of local unique ID
                                          with an offset equal to number of
                                          bytes in the received unique ID

        Rule E. On an Allocation message WHERE (source node ID is
        non-anonymous) AND (unique_id fully matches client's unique ID) AND
        (node_id in the received message is not zero):
        1) Request Timer stops.
        2) The client initializes its node_id with the received value.
        3) The client terminates subscription to Allocation messages.
        4) Exit.
        */

        /* Count the number of unique ID bytes matched */
        for (i = offset; i < rx_len && unique_id_matched < 16u &&
                hw_version.unique_id[unique_id_matched] == rx_payload[i];
            unique_id_matched++, i++);

        if (i < rx_len) {
            /* Abort if we didn't match the whole unique ID */
            server_node_id = 0u;
        } else if (rx_frame_id.last_frame && unique_id_matched < 16u) {
            /* Case D */
            uavcan_tx_allocation_message(node_id, 16u, hw_version.unique_id,
                                         unique_id_matched, transfer_id++);
        } else if (rx_frame_id.last_frame) {
            /* Validate CRC */
            expected_crc = UAVCAN_ALLOCATION_CRC;
            expected_crc = crc16_add(expected_crc, server_allocated_node_id);
            expected_crc = crc16_signature(expected_crc, 16u,
                                           hw_version.unique_id);

            /* Case E */
            if (rx_crc == expected_crc) {
                node_id = server_allocated_node_id >> 1u;
            }
        }
    } while (node_id == 0u && !g_bootloader_tboot_expired);

    return node_id;
}


void bootloader_poll_getnodeinfo(
    uint8_t node_id,
    uint32_t uptime,
    uint8_t status
) {
    uavcan_getnodeinfo_response_t response;
    uavcan_nodestatus_t node_status;
    uavcan_frame_id_t frame_id;
    size_t frame_len, i;
    uint32_t rx_message_id;
    uint8_t frame_payload[8], got_frame;

    /* UAVCANBootloader_v0.3 #21.2.1.2: [AppValid]:SetAppVersion(SwVerion) */
    node_status.uptime_sec = uptime;
    node_status.status_code = status;
    node_status.vendor_specific_status_code = 0u;
    uavcan_pack_nodestatus(response.nodestatus, &node_status);

    board_get_hardware_version(&response.hardware_version);
    response.name_length = board_get_product_name(response.name);

    if (g_bootloader_app_valid) {
        response.software_version.major =
            g_fw_image_descriptor->major_version;
        response.software_version.minor =
            g_fw_image_descriptor->minor_version;
        response.software_version.optional_field_mask = 3u;
            g_fw_image_descriptor->minor_version;
        response.software_version.vcs_commit =
            g_fw_image_descriptor->vcs_commit;
        response.software_version.image_crc =
            g_fw_image_descriptor->image_crc;
    } else {
        for (i = 0u; i < sizeof(response.software_version); i++) {
            *(uint8_t*)(&response.software_version) = 0u;
        }
    }

    /*
    Receive from FIFO 1 -- filters are configured to push the messages there,
    and this is called from SysTick so needs to avoid the same FIFOs/mailboxes
    as the rest of the application.
    */
    rx_message_id = 0u;
    got_frame = can_rx(&rx_message_id, &frame_len, frame_payload, 1u);
    uavcan_parse_message_id(&frame_id, rx_message_id);
    if (got_frame && frame_id.data_type_id == UAVCAN_GETNODEINFO_DTID &&
            frame_payload[0] == node_id &&
            frame_id.last_frame) {
        /* UAVCANBootloader_v0.3 #21.2.1.3: 551.GetNodeInfo.uavcan */
        uavcan_tx_getnodeinfo_response(node_id, &response,
                                       frame_id.source_node_id,
                                       frame_id.transfer_id);

        g_bootloader_got_node_info = 1u;
    }
}


void bootloader_wait_for_beginfirmwareupdate(
    uint8_t *fw_path,
    uint8_t *fw_path_length,
    uint8_t *fw_source_node_id
) {
    uavcan_beginfirmwareupdate_request_t request;
    uavcan_frame_id_t frame_id;
    size_t i;
    uint8_t frame_payload[8];
    can_error_t status;

    status = CAN_ERROR;
    while (!g_bootloader_tboot_expired && status != CAN_OK) {
        status = uavcan_rx_beginfirmwareupdate_request(g_bootloader_node_id,
                                                       &request, &frame_id);
    }

    if (status == CAN_OK) {
        /* UAVCANBootloader_v0.3 #22.2: Resp580.BeginFirmwareUpdate.uavcan */

        /* Send an ERROR_OK response */
        frame_payload[0] = frame_id.source_node_id;
        frame_payload[1] = 0u;

        frame_id.last_frame = 1u;
        frame_id.frame_index = 0u;
        frame_id.source_node_id = g_bootloader_node_id;
        frame_id.transfer_type = SERVICE_RESPONSE;

        can_tx(uavcan_make_message_id(&frame_id), 2u, frame_payload, 0u);

        /* UAVCANBootloader_v0.3 #22.3: fwPath = image_file_remote_path */
        for (i = 0u; i < request.path_length; i++) {
            fw_path[i] = request.path[i];
        }
        *fw_path_length = request.path_length;
        /* UAVCANBootloader_v0.3 #22.4: fwSourceNodeID = source_node_id */
        *fw_source_node_id = request.source_node_id;
    } else {
        fw_path[0] = 0;
        *fw_source_node_id = 0;
    }
}


void bootloader_file_getinfo(
    size_t *fw_image_size,
    uint64_t *fw_image_crc,
    const uint8_t *fw_path,
    uint8_t fw_path_length,
    uint8_t fw_source_node_id
) {
    uavcan_getinfo_request_t request;
    uavcan_getinfo_response_t response;
    uint8_t transfer_id, retries, i;
    can_error_t status;

    for (i = 0; i < fw_path_length; i++) {
        request.path[i] = fw_path[i];
    }
    request.path_length = fw_path_length;

    /* UAVCANBootloader_v0.3 #25: SetRetryAndTimeout(3,1000MS) */
    retries = UAVCAN_SERVICE_RETRIES;
    transfer_id = 0;

    *fw_image_size = 0;
    *fw_image_crc = 0;

    while (retries) {
        /* UAVCANBootloader_v0.3 #26: 585.GetInfo.uavcan(path) */
        uavcan_tx_getinfo_request(g_bootloader_node_id, &request,
                                  fw_source_node_id, transfer_id);

        /* UAVCANBootloader_v0.3 #28: 585.GetInfo.uavcan(fw_path,fw_crc,fw_size...), */
        status = uavcan_rx_getinfo_response(
            g_bootloader_node_id, &response, fw_source_node_id,
            transfer_id, UAVCAN_SERVICE_TIMEOUT_TICKS);

        transfer_id++;

        /* UAVCANBootloader_v0.3 #27: validateFileInfo(file_info, &errorcode) */
        if (status == CAN_OK && response.error == UAVCAN_FILE_ERROR_OK &&
                (response.entry_type & UAVCAN_GETINFO_ENTRY_TYPE_FLAG_FILE) &&
                (response.entry_type & UAVCAN_GETINFO_ENTRY_TYPE_FLAG_READABLE) &&
                response.size > 0u && response.size < OPT_APPLICATION_IMAGE_LENGTH) {
            /* UAVCANBootloader_v0.3 #28.4: save(file_info) */
            *fw_image_size = response.size;
            *fw_image_crc = response.crc64;
            break;
        } else {
            retries--;
        }
    }
}


flash_error_t bootloader_file_read_and_program(
    uint8_t fw_source_node_id,
    uint8_t fw_path_length,
    const uint8_t *fw_path,
    size_t fw_image_size,
    uint32_t *fw_word0
) {
    uavcan_read_request_t request;
    uavcan_read_response_t response;
    size_t bytes_written, i, write_length;
    can_error_t can_status;
    flash_error_t flash_status;
    uint32_t next_read_deadline;
    uint8_t transfer_id, retries, write_remainder[4], write_remainder_length,
            *data;

    /* Set up the read request */
    for (i = 0; i < fw_path_length; i++) {
        request.path[i] = fw_path[i];
    }
    request.path_length = fw_path_length;
    request.offset = 0u;

    bytes_written = 0u;
    write_remainder_length = 0u;
    transfer_id = 0u;
    next_read_deadline = 0u;

    do {
        /*
        TODO
        Rate limiting on read requests:
        - 2/sec on a 125 Kbaud bus
        - 4/sec on a 250 Kbaud bus
        - 8/sec on a 500 Kbaud bus
        - 16/sec on a 1 Mbaud bus
        */
        while (g_bootloader_uptime < next_read_deadline);
        next_read_deadline = g_bootloader_uptime + 6u;

        /* UAVCANBootloader_v0.3 #28.11: SetRetryAndTimeout(3,1000MS) */
        retries = UAVCAN_SERVICE_RETRIES;
        can_status = CAN_ERROR;
        while (retries && can_status != CAN_OK) {
            /* UAVCANBootloader_v0.3 #28.12: 588.Read.uavcan(0, path) */
            /* UAVCANBootloader_v0.3 #33: 588.Read.uavcan(N,path) */
            request.offset = bytes_written + write_remainder_length;
            uavcan_tx_read_request(g_bootloader_node_id, &request,
                                   fw_source_node_id, transfer_id);

            /* UAVCANBootloader_v0.3 #28.12.1: 588.Read.uavcan(0,path,data) */
            /* UAVCANBootloader_v0.3 #33.1: 583.Read.uavcan(0,path,data) */
            can_status = uavcan_rx_read_response(g_bootloader_node_id,
                                                 &response, fw_source_node_id,
                                                 transfer_id,
                                                 UAVCAN_SERVICE_TIMEOUT_TICKS);

            transfer_id++;

            /* UAVCANBootloader_v0.3 #34: ValidateReadResp(resp) */
            if (can_status != CAN_OK ||
                    response.error != UAVCAN_FILE_ERROR_OK) {
                can_status = CAN_ERROR;

                /* UAVCANBootloader_v0.3 #35: [(retries != 0 && timeout) || !ValidReadReadResponse]:INDICATE_FW_UPDATE_INVALID_RESPONSE */
                board_indicate_fw_update_invalid_response();

                /* UAVCANBootloader_v0.3 #36: [(retries != 0 && timeout) || !ValidReadReadResponse]:1023.LogMessage.uavcan */
                bootloader_send_log_message(g_bootloader_node_id,
                                            UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                            UAVCAN_LOGMESSAGE_STAGE_PROGRAM,
                                            UAVCAN_LOGMESSAGE_RESULT_FAIL);
                retries--;
            }
        }

        if (can_status != CAN_OK) {
            /* UAVCANBootloader_v0.3 #37: [(retries == 0 && timeout]:INDICATE_FW_UPDATE_TIMEOUT */
            board_indicate_fw_update_timeout();
            break;
        }

        data = response.data;
        write_length = response.data_length;

        /*
        If this is the last read (zero length) and there are bytes left to
        write, pad the firmware image out with zeros to ensure a full-word
        write.
        */
        if (write_length == 0u && write_remainder_length > 0u) {
            write_length = 4u - write_remainder_length;
            data[0] = data[1] = data[2] = 0u;
        }

        /*
        If the length of a previous read was not a multiple of 4, we'll
        have a few bytes left over which need to be combined with the
        next write as all writes must be word-aligned and a whole number
        of words long.
        */
        flash_status = FLASH_OK;
        while (write_length && flash_status == FLASH_OK) {
            write_remainder[write_remainder_length++] = *(data++);
            write_length--;

            if (write_remainder_length == 4u) {
                if (bytes_written == 0u) {
                    /* UAVCANBootloader_v0.3 #30: SaveWord0 */
                    ((uint8_t*)fw_word0)[0] = write_remainder[0];
                    ((uint8_t*)fw_word0)[1] = write_remainder[1];
                    ((uint8_t*)fw_word0)[2] = write_remainder[2];
                    ((uint8_t*)fw_word0)[3] = write_remainder[3];
                } else {
                    flash_status = flash_write_word(
                        (uint32_t)(&g_fw_image[bytes_written >> 2u]),
                        write_remainder);
                }

                bytes_written += 4u;
                write_remainder_length = 0u;
            }
        }
    } while (bytes_written <= fw_image_size && response.data_length != 0u &&
             flash_status == FLASH_OK);

    /*
    Return success iff the last read succeeded, the last write succeeded, the
    correct number of bytes were written, and the length of the last response
    was zero.
    */
    if (can_status == CAN_OK && flash_status == FLASH_OK &&
            bytes_written == fw_image_size && response.data_length == 0u) {
        return FLASH_OK;
    } else {
        return FLASH_ERROR;
    }
}


void bootloader_send_log_message(
    uint8_t node_id,
    uint8_t level,
    uint8_t stage,
    uint8_t status
) {
    uavcan_logmessage_t message;
    uavcan_frame_id_t frame_id;
    uint8_t payload[8];
    size_t frame_len;

    frame_id.transfer_id = 0;
    frame_id.last_frame = 1u;
    frame_id.frame_index = 0;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = MESSAGE_BROADCAST;
    frame_id.data_type_id = UAVCAN_LOGMESSAGE_DTID;

    message.level = level;
    message.message[0] = stage;
    message.message[1] = status;
    frame_len = uavcan_pack_logmessage(payload, &message);
    can_tx(uavcan_make_message_id(&frame_id), frame_len, payload, 0u);
}


uint8_t bootloader_timeout(uint32_t start_ticks, uint32_t ticks) {
    if (start_ticks > 0) {
        return (g_bootloader_uptime - start_ticks) >= ticks;
    } else {
        return 0;
    }
}


uint8_t bootloader_is_app_valid(uint32_t first_word) {
    uint64_t crc;
    size_t i, length, crc_offset;
    uint8_t byte;

    bootloader_find_descriptor();

    /* UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) && ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
    if (!g_fw_image_descriptor || first_word == 0xFFFFFFFFu) {
        return 0u;
    }

    length = g_fw_image_descriptor->image_size;
    crc_offset = (size_t)(&g_fw_image_descriptor->image_crc) -
                 (size_t)g_fw_image;

    crc = CRC64_INITIAL;
    for (i = 0u; i < 4u; i++) {
        crc = crc64_add(crc, (uint8_t)(first_word >> (i << 3u)));
    }
    for (i = 4u; i < length; i++) {
        if (crc_offset <= i && i < crc_offset + 8u) {
            /* Zero out the CRC field while computing the CRC */
            byte = 0u;
        } else {
            byte = ((volatile uint8_t*)g_fw_image)[i];
        }
        crc = crc64_add(crc, byte);
    }
    crc ^= CRC64_OUTPUT_XOR;

    return crc == g_fw_image_descriptor->image_crc;
}


void bootloader_find_descriptor(void) {
    size_t i, addr;

    g_fw_image_descriptor = NULL;
    for (i = 0u; i < FLASH_SIZE / 4u - OPT_APPLICATION_IMAGE_OFFSET - 1u; i++) {
        if (g_fw_image[i] == APP_DESCRIPTOR_SIGNATURE_LO &&
                g_fw_image[i + 1u] == APP_DESCRIPTOR_SIGNATURE_HI) {
            addr = (size_t)&g_fw_image[i];
            g_fw_image_descriptor = (volatile app_descriptor_t*)addr;
            break;
        }
    }
}


void
__attribute__((externally_visible,noinline,noreturn))
application_run(void) {
    void (*const application)(void) =
        (void (*)(void))(g_fw_image[1]);

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    __disable_irq();
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

    __DSB(); /* Memory synchronisation barrier just in case */
    __ISB(); /* Flush CPU instruction pipeline */
    SCB->VTOR = OPT_APPLICATION_IMAGE_OFFSET; /* Update the vector offset */
    __DSB(); /* Make sure the vector table offset write has finished */

    /* Set up the application SP */
    __set_MSP(g_fw_image[0]);

    /* Jump to the application's reset handler */
    application();

    while (1);
}
