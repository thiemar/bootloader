#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "cmsis_device.h"
#include "bootloader_config.h"
#include "flash.h"
#include "uavcan.h"
#include "crc.h"


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
void bootloader_poll_getnodeinfo(void);
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
    size_t fw_image_size
);
uint8_t bootloader_is_app_valid(uint8_t ignore_magic);

volatile uint8_t g_bootloader_node_id;
volatile uint8_t g_bootloader_status_code;
volatile uint8_t g_bootloader_got_node_info;
volatile uint8_t g_bootloader_app_valid;
volatile uint32_t g_bootloader_uptime;

volatile uint32_t g_bootloader_tboot_deadline;
volatile uint8_t g_bootloader_tboot_expired;
volatile uint8_t g_bootloader_tboot_enable;

volatile fw_image_descriptor_t *g_fw_image_descriptor;

/*
SysTick task executes at 100 Hz and performs the following actions:
* Counts uptime seconds in g_bootloader_uptime;
* Sets g_bootloader_tboot_timeout to 1 once Tboot has elapsed;
* Sends uavcan.protocol.NodeStatus at 100 Hz
* Replies to uavcan.protocol.GetNodeInfo requests

UAVCANBootloader_v0.3 #24: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
UAVCANBootloader_v0.3 #28.5: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
UAVCANBootloader_v0.3 #49: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING)
*/
void SysTick_Handler(void) {
    /* Update uptime and check for Tboot expiry */
    g_bootloader_uptime++;
    if (g_bootloader_uptime > g_bootloader_tboot_deadline &&
            g_bootloader_tboot_enable) {
        g_bootloader_tboot_expired = 1u;
    }

    if (g_bootloader_node_id) {
        /* Handle GetNodeInfo replies regardless of application state */
        bootloader_poll_getnodeinfo();

        /* Send uavcan.protocol.NodeStatus every 50 ticks */
        if (g_bootloader_uptime % UAVCAN_NODESTATUS_INTERVAL_TICKS == 0) {
            uavcan_tx_nodestatus(g_bootloader_node_id,
                                 g_bootloader_uptime / 100u,
                                 g_bootloader_status_code);
        }
    }
}


void
__attribute__((noreturn))
bootloader_main(void) {
    uint64_t fw_image_crc;
    size_t fw_image_size;
    uint32_t restart_ticks;
    uint8_t fw_path[200], fw_path_length, fw_source_node_id, fw_magic[4];
    uint8_t app_bl_request, wait_for_getnodeinfo, error_log_stage;
    flash_error_t status;

    /* UAVCANBootloader_v0.3 #1: Reset */
    g_fw_image_descriptor = (volatile fw_image_descriptor_t *)(FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET);

    /* UAVCANBootloader_v0.3 #2: Clock, IO Init */
    board_initialize();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* TODO UAVCANBootloader_v0.3 #4: EnableWatchDog */
#ifdef OPT_ENABLE_WD
    /* ... */
#endif

    /* UAVCANBootloader_v0.3 #6: INDICATE_RESET */
    board_indicate_reset();

    /* UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) && ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
    g_bootloader_app_valid = bootloader_is_app_valid(0u);

    /* TODO UAVCANBootloader_v0.3 #8: InitNodeInfo(AppValid) ? */
    g_bootloader_got_node_info = 0u;


    /* TODO UAVCANBootloader_v0.3 #9: bool AppBLRequest = Validate(BootLoaderAppCommon, AP) */
    app_bl_request = 0u;

    /* TODO UAVCANBootloader_v0.3 #10: Init(BootLoaderAppCommon, BL, AppBLRequest) */
    g_bootloader_node_id = 0u;

    /* UAVCANBootloader_v0.3 #11: bool WaitForGetNodeInfo = -DOPT_WAIT_FOR_GET_NODE_INFO */
#ifdef OPT_WAIT_FOR_GETNODEINFO
    wait_for_getnodeinfo = 1u;
#else
    wait_for_getnodeinfo = 0u;
#endif

    /* TODO UAVCANBootloader_v0.3 #12: [ -DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO] : WaitForGetNodeInfo = IORead(-DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO) */
#ifdef OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO
    /* ... */
#endif

    /* UAVCANBootloader_v0.3 #13: [ !WaitForGetNodeInfo && AppValid]: StartTboot([-DOPT_TBOOT) */
    g_bootloader_tboot_enable = 0u;
    g_bootloader_tboot_expired = 0u;
    g_bootloader_tboot_deadline = 0u;
    g_bootloader_status_code = 0u;
    g_bootloader_uptime = 0u;
    if (!wait_for_getnodeinfo && !app_bl_request && g_bootloader_app_valid) {
        g_bootloader_tboot_enable = 1u;
        g_bootloader_tboot_deadline = OPT_TBOOT_MS / 10u;
    }

    /* Start SysTick interrupt for Tboot and NodeStatus messages */
    SysTick->LOAD = OPT_CPU_FREQ_HZ / 100u - 1u;
    SysTick->VAL = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;

    /* UAVCANBootloader_v0.3 #14: [AppBLRequest]:RestartFromApp */
    if (!app_bl_request || !g_bootloader_node_id) {
        bootloader_autobaud_and_get_dynamic_node_id();
        if (g_bootloader_tboot_expired) {
            goto boot;
        } else {
            /* TODO UAVCANBootloader_v0.3 #19.1: SetNodeID(newNodeID), Update(BootLoaderAppCommon, BL) */
        }
    }

    /* UAVCANBootloader_v0.3 #21: RestartFromApp */

    /* UAVCANBootloader_v0.3 #21.1: initSeconds(0) */
    g_bootloader_uptime = 0u;

    /* UAVCANBootloader_v0.3 #21.2: 550.NodeStatus.uavcan(uptime=0, STATUS_INITIALIZING) */
    g_bootloader_status_code = UAVCAN_NODESTATUS_STATUS_INITIALIZING;
    uavcan_tx_nodestatus(g_bootloader_node_id, g_bootloader_uptime,
                         g_bootloader_status_code);

    /* UAVCANBootloader_v0.3 #21.2.1: Req551.GetNodeInfo.uavcan() */
    while (!g_bootloader_got_node_info && !g_bootloader_tboot_expired);

    /* UAVCANBootloader_v0.3 #21.2.1.4: [Timeout(Tboot)]: jump_to_app */
    if (g_bootloader_tboot_expired) {
        goto boot;
    }

    /* UAVCANBootloader_v0.3 #21.2.1.1: [(AppBLRequest || WaitForGetNodeInfo) && AppValid)]: StartTboot([-DOPT_TBOOT) */
    if ((app_bl_request || wait_for_getnodeinfo) && g_bootloader_app_valid) {
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
    if (fw_image_size < sizeof(fw_image_descriptor_t) || !fw_image_crc) {
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
                                              fw_image_size);
    if (status != FLASH_OK) {
        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_PROGRAM;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #41: CalulateCRC(file_info) */
    if (!bootloader_is_app_valid(1u)) {
        g_bootloader_app_valid = 0u;

        /* UAVCANBootloader_v0.3 #43: [crc Fail]:INDICATE_FW_UPDATE_INVALID_CRC */
        board_indicate_fw_update_invalid_crc();

        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_VALIDATE;
        goto failure;
    }

    /* UAVCANBootloader_v0.3 #46: [crc == fw_crc]:write word0 */
    fw_magic[0] = (uint8_t)OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC;
    fw_magic[1] = (uint8_t)(OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC >> 8u);
    fw_magic[2] = (uint8_t)(OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC >> 16u);
    fw_magic[3] = (uint8_t)(OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC >> 24u);
    status = flash_write_data(FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET, 4u,
                              fw_magic);
    if (status != FLASH_OK) {
        /* Not specifically listed in UAVCANBootloader_v0.3: 1023.LogMessage.uavcan */
        error_log_stage = UAVCAN_LOGMESSAGE_STAGE_PROGRAM;
        goto failure;
    }

    /* TODO UAVCANBootloader_v0.3 #47: ValidateBootLoaderAppCommon() */

    /* TODO UAVCANBootloader_v0.3 #48: KickTheDog() */

    /* TODO UAVCANBootloader_v0.3 #50: jump_to_app */

boot:
    while(1);

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

    /* TODO UAVCANBootloader_v0.3 #28.1: [Retries==0]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
    /* TODO UAVCANBootloader_v0.3 #40: [Retries==0]:InvalidateBootLoaderAppCommon(), RestartWithDelay(20,000ms) */
    /* TODO UAVCANBootloader_v0.3 #45: [crc Fail]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
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


uint64_t bootloader_get_short_unique_id(void) {
    uavcan_hardwareversion_t hw_version;
    uint64_t result;
    size_t i;

    board_get_hardware_version(&hw_version);

    result = CRC64_INITIAL;
    for (i = 0; i < 16u; i++) {
        result = crc64_add(result, hw_version.unique_id[i]);
    }

    return (result ^ CRC64_OUTPUT_XOR) & 0x01FFFFFFFFFFFFFFL;
}


void bootloader_autobaud_and_get_dynamic_node_id(void) {
    /* UAVCANBootloader_v0.3 #15: [!AppBLRequest]:INDICATE_AUTOBAUDING */
    board_indicate_autobaud_start();

    /* UAVCANBootloader_v0.3 #16: [!AppBLRequest]:Auto Baud */
    /* UAVCANBootloader_v0.3 #17: [!AppBLRequest]:detect bitrate */
    uavcan_autobaud(); /* does UAVCANBootloader_v0.3 #18.1 */

    /* UAVCANBootloader_v0.3 #18.4: [Timeout(Tboot)]: jump_to_app (node_id is 0 => App need to Allocate or use Static NodeID */
    if (g_bootloader_tboot_expired) {
        return;
    }

    /* TODO UAVCANBootloader_v0.3 #18.2: [!AppBLRequest]:Update(BootLoaderAppCommon, BL) */

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
    uavcan_dynamicnodeidallocation_t allocation_msg, allocation_response;
    uavcan_frame_id_t tx_frame_id, rx_frame_id;
    uint32_t last_request_ticks;
    uint8_t tx_frame_payload[8], rx_frame_payload[8];
    size_t tx_frame_len, rx_frame_len;
    uint8_t node_id, got_frame;

    node_id = 0;
    got_frame = 0;

    /* UAVCANBootloader_v0.3 #18.6: GetUUID() */
    allocation_msg.short_unique_id = bootloader_get_short_unique_id();
    allocation_msg.node_id = node_id;

    /* Prepare the DynamicNodeIDAllocation request frame */
    tx_frame_id.transfer_id = 0u;
    tx_frame_id.last_frame = 1u;
    tx_frame_id.frame_index = 0u;
    tx_frame_id.source_node_id = 0u;
    tx_frame_id.transfer_type = MESSAGE_BROADCAST;
    tx_frame_id.data_type_id = UAVCAN_DYNAMICNODEIDALLOCATION_DTID;

    tx_frame_len = uavcan_pack_dynamicnodeidallocation(
        tx_frame_payload, &allocation_msg);

    /*
    This is repeated indefinitely, rate limited at 0.5 message per second
    and will exit to application if Timeout(Tboot) occurs.

    A Timeout(Tboot) will never occur if !AppValid
    */
    do {
        /* UAVCANBootloader_v0.3 #18.7: Req559.DynamicNodeIDAllocation.uavcan(uuid,0) */
        tx_frame_id.transfer_id++;
        uavcan_tx(&tx_frame_id, tx_frame_len, tx_frame_payload, 0u);

        /*
        FIXME: Rate limit is set to 2 messages per second. A rate limit
        of 0.5 messages per second would mean only one message is sent
        during the default Tboot.
        */
        last_request_ticks = g_bootloader_uptime;
        while (!g_bootloader_tboot_expired &&
                !bootloader_timeout(last_request_ticks, 50u)) {
            got_frame = uavcan_rx(&rx_frame_id, &rx_frame_len,
                                  rx_frame_payload, 0u);
            if (got_frame && rx_frame_id.data_type_id ==
                        UAVCAN_DYNAMICNODEIDALLOCATION_DTID) {
                uavcan_unpack_dynamicnodeidallocation(&allocation_response,
                                                      rx_frame_payload);
                if (allocation_response.short_unique_id ==
                        allocation_msg.short_unique_id) {
                    node_id = allocation_response.node_id;
                    break;
                }
            }
        }

        if (g_bootloader_tboot_expired) {
            break;
        }
    } while (node_id == 0u);

    return node_id;
}


void bootloader_poll_getnodeinfo(void) {
    uavcan_getnodeinfo_response_t response;
    uavcan_nodestatus_t node_status;
    uavcan_frame_id_t frame_id;
    size_t frame_len;
    uint8_t frame_payload[8], got_frame;

    /*
    Receive from FIFO 1 -- filters are configured to push the messages there,
    and this is called from SysTick so needs to avoid the same FIFOs/mailboxes
    as the rest of the application.
    */
    got_frame = uavcan_rx(&frame_id, &frame_len, frame_payload, 1u);
    if (got_frame && frame_id.data_type_id == UAVCAN_GETNODEINFO_DTID &&
            frame_payload[0] == g_bootloader_node_id &&
            frame_id.last_frame) {
        /* UAVCANBootloader_v0.3 #21.2.1.2: [AppValid]:SetAppVersion(SwVerion) */
        node_status.uptime_sec = g_bootloader_uptime;
        node_status.status_code = g_bootloader_status_code;
        node_status.vendor_specific_status_code = 0u;
        uavcan_pack_nodestatus(response.nodestatus, &node_status);
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
            response.software_version.major = 0u;
            response.software_version.minor = 0u;
            response.software_version.optional_field_mask = 0u;
            response.software_version.vcs_commit = 0u;
            response.software_version.image_crc = 0u;
        }

        board_get_hardware_version(&response.hardware_version);
        response.name_length = board_get_product_name(response.name);

        /* UAVCANBootloader_v0.3 #21.2.1.3: 551.GetNodeInfo.uavcan */
        uavcan_tx_getnodeinfo_response(g_bootloader_node_id, &response,
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

        uavcan_tx(&frame_id, 2u, frame_payload, 0u);

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
    size_t fw_image_size
) {
    uavcan_read_request_t request;
    uavcan_read_response_t response;
    size_t bytes_written, i, write_length;
    can_error_t can_status;
    flash_error_t flash_status;
    uint8_t transfer_id, retries, write_remainder[4], write_remainder_length,
            *data;

    /* Set up the read request */
    for (i = 0; i < fw_path_length; i++) {
        request.path[i] = fw_path[i];
    }
    request.path_length = fw_path_length;

    /* Don't write the magic word yet */
    request.offset = bytes_written = 4u;
    write_remainder_length = 0u;
    transfer_id = 0u;

    while (bytes_written <= fw_image_size) {
        /* TODO: add rate limiting on read requests -- 5/sec */
        while (0);

        /* UAVCANBootloader_v0.3 #28.11: SetRetryAndTimeout(3,1000MS) */
        retries = UAVCAN_SERVICE_RETRIES;
        can_status = CAN_ERROR;
        while (retries && can_status != CAN_OK) {
            /* UAVCANBootloader_v0.3 #28.12: 588.Read.uavcan(0, path) */
            /* UAVCANBootloader_v0.3 #33: 588.Read.uavcan(N,path) */
            request.offset = bytes_written;
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

            return FLASH_ERROR;
        } else if (response.data_length) {
            /*
            Not necessary in this implementation:
            UAVCANBootloader_v0.3 #30: SaveWord0
            */
            data = response.data;
            write_length = response.data_length;

            /*
            If the length of a previous read was not a multiple of 4, we'll
            have a few bytes left over which need to be combined with the
            next write as all writes must be word-aligned and a whole number
            of words long. Here we steal up to 3 bytes from the current
            read in order to finish the previous write, and adjust the
            starting address and length of the current write to compensate.
            */
            if (write_remainder_length) {
                for (i = write_remainder_length;
                        i < 4u && write_length; i++, write_length--) {
                    write_remainder[i] = *(data++);
                }

                flash_status = flash_write_data(
                    FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET + bytes_written,
                    4u, write_remainder);
                bytes_written += 4u;

                if (flash_status != FLASH_OK) {
                    return flash_status;
                }
            }

            /*
            If the current write is not a multiple of 4 long, save the
            trailing bytes to the write remainder so they can be written next
            time.
            */
            write_remainder_length = write_length & 3u;
            write_length -= write_remainder_length;
            for (i = write_length; i < write_remainder_length; i++) {
                write_remainder[i - write_length] = data[i];
            }

            if (write_length) {
                flash_status = flash_write_data(
                    FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET + bytes_written,
                    write_length, data);
                bytes_written += write_length;

                if (flash_status != FLASH_OK) {
                    return flash_status;
                }
            }
        } else {
            /*
            If there was a write remainder from the last read operation,
            pad it with zeros and finish it up now.
            */
            if (write_remainder_length) {
                for (i = write_remainder_length; i < 4u; i++) {
                    write_remainder[i] = 0u;
                }

                flash_status = flash_write_data(
                    FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET + bytes_written,
                    4u, write_remainder);
                bytes_written += write_remainder_length;

                if (flash_status != FLASH_OK) {
                    return flash_status;
                }
            }

            /*
            Read to EOF: return success if the correct number of bytes were
            written, and failure otherwise
            */
            return bytes_written == fw_image_size ? FLASH_OK : FLASH_ERROR;
        }
    }

    /*
    We should have exited via the 'else' in the loop above -- this means the
    file being read is longer than the server told us originally.
    */
    return FLASH_ERROR;
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
    uavcan_tx(&frame_id, frame_len, payload, 0u);
}


uint8_t bootloader_timeout(uint32_t start_ticks, uint32_t ticks) {
    if (start_ticks > 0) {
        return (g_bootloader_uptime - start_ticks) >= ticks;
    } else {
        return 0;
    }
}


uint8_t bootloader_is_app_valid(uint8_t ignore_magic) {
    uint64_t crc;
    uint8_t i, byte;

    /* UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) && ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
    if (!ignore_magic && g_fw_image_descriptor->descriptor_magic !=
            OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC) {
        return 0u;
    }

    crc = CRC64_INITIAL;
    for (i = 0u; i < sizeof(fw_image_descriptor_t); i++) {
        byte = ((volatile uint8_t*)g_fw_image_descriptor)[i];
        if (i < 4u) {
            /*
            Fill the first four bytes with the descriptor magic because that's
            just been checked if required -- means we can use this function
            to validate the application image before the magic word has been
            written.
            */
            byte = (uint8_t)(OPT_APPLICATION_IMAGE_DESCRIPTOR_MAGIC >> (i << 3u));
        } else if (8u <= i && i < 16u) {
            /* Zero out the CRC field while computing the CRC */
            byte = 0u;
        }
        crc = crc64_add(crc, byte);
    }

    crc = flash_crc(
        FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET +
            sizeof(fw_image_descriptor_t),
        g_fw_image_descriptor->image_size - 128u, crc);

    return crc == g_fw_image_descriptor->image_crc;
}
