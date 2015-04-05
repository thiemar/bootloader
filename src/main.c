#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "cmsis_device.h"
#include "bootloader_config.h"
#include "flash.h"
#include "uavcan.h"


typedef enum {
  GPIO_Speed_Level_1  = 1,
  GPIO_Speed_Level_2  = 2,
  GPIO_Speed_Level_3  = 3
} gpio_speed_t;

typedef enum {
  GPIO_OType_PP = 0,
  GPIO_OType_OD = 1
} gpio_otype_t;

typedef enum {
  GPIO_Mode_IN   = 0,
  GPIO_Mode_OUT  = 1,
  GPIO_Mode_AF   = 2,
  GPIO_Mode_AN   = 3
} gpio_mode_t;

typedef enum {
  GPIO_PuPd_NOPULL = 0,
  GPIO_PuPd_UP     = 1,
  GPIO_PuPd_DOWN   = 2
} gpio_pupd_t;


void bootloader_gpio_init(void);
void bootloader_reset(void);
uint64_t bootloader_get_short_unique_id(void);
void bootloader_send_node_status(
    uint8_t node_id,
    uint32_t uptime_sec,
    uint8_t status_code
);
void bootloader_send_log_message(
    uint8_t node_id,
    uint8_t level,
    uint8_t stage,
    uint8_t status
);
uint8_t bootloader_timeout(uint32_t start_cyccnt, uint32_t millis);

volatile uint8_t g_bootloader_tboot_timeout;
volatile uint8_t g_bootloader_node_id;
volatile uint8_t g_bootloader_status_code;
volatile uint32_t g_bootloader_uptime;

/*
TODO:
SysTick task executing at 100 Hz or so to count uptime seconds in
g_bootloader_uptime, and set g_bootloader_tboot_timeout to 1 once Tboot has
elapsed.
*/


void
__attribute__((noreturn))
bootloader_main(void) {
    uavcan_frame_id_t tx_frame_metadata, rx_frame_metadata;
    uint32_t tx_frame_payload[2], tx_frame_id, rx_frame_payload[2],
             rx_frame_id, uptime_sec;
    size_t tx_frame_len, rx_frame_len;
    uint8_t got_frame;

    /* UAVCANBootloader_v0.3 #1: Reset */

    /* FIXME UAVCANBootloader_v0.3 #2: Clock, IO Init */
    SystemInit();
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    bootloader_gpio_init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* TODO UAVCANBootloader_v0.3 #4: EnableWatchDog */
    /* TODO UAVCANBootloader_v0.3 #6: INDICATE_RESET */

    uint8_t app_bl_request, app_valid, wait_for_getnodeinfo;
    /* TODO UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) && ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
    app_valid = 0;

    /* TODO UAVCANBootloader_v0.3 #8: InitNodeInfo(AppValid) */

    /* TODO UAVCANBootloader_v0.3 #9: bool AppBLRequest = Validate(BootLoaderAppCommon, AP) */
    app_bl_request = 0;

    /* TODO UAVCANBootloader_v0.3 #10: Init(BootLoaderAppCommon, BL, AppBLRequest) */
    g_bootloader_node_id = 0;

    /* TODO UAVCANBootloader_v0.3 #11: bool WaitForGetNodeInfo = -DOPT_WAIT_FOR_GET_NODE_INFO */
#ifdef OPT_WAIT_FOR_GETNODEINFO
    wait_for_getnodeinfo = 1;
#else
    wait_for_getnodeinfo = 0;
#endif

    /* TODO UAVCANBootloader_v0.3 #12: [ -DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO] : WaitForGetNodeInfo = IORead(-DOPT_W AIT_FOR_GET_NODE_INFO_JUMPER_GPIO) */

    /* TODO UAVCANBootloader_v0.3 #13: [ !WaitForGetNodeInfo && AppValid]: StartTboot([-DOPT_TBOOT) */
    g_bootloader_tboot_timeout = 0;  /* no C standard init code */
    if (!wait_for_getnodeinfo && app_valid) {
        /* Start SysTick interrupt for Tboot */
    }

    /* UAVCANBootloader_v0.3 #14: [AppBLRequest]:RestartFromApp */
    if (!app_bl_request) {
        /* TODO UAVCANBootloader_v0.3 #15: [!AppBLRequest]:INDICATE_AUTOBAUDING */

        /* UAVCANBootloader_v0.3 #16: [!AppBLRequest]:Auto Baud */
        /* UAVCANBootloader_v0.3 #17: [!AppBLRequest]:detect bitrate */
        uavcan_autobaud(); /* does UAVCANBootloader_v0.3 #18.1 */

        /* UAVCANBootloader_v0.3 #18.4: [Timeout(Tboot)]: jump_to_app (node_id is 0 => App need to Allocate or use Static NodeID */
        if (g_bootloader_tboot_timeout) {
            goto boot;
        }

        /* TODO UAVCANBootloader_v0.3 #18.2: [!AppBLRequest]:Update(BootLoaderAppCommon, BL) */

        /* TODO UAVCANBootloader_v0.3 #18.3: [!AppBLRequest]:INDICATE_AUTOBAUDED */

        /* TODO UAVCANBootloader_v0.3 #18.5: [!AppBLRequest]:INDICATE_ALLOCATION */

        /* UAVCANBootloader_v0.3 #18.6: GetUUID() */
        uavcan_dynamicnodeidallocation_t allocation_msg, allocation_response;
        allocation_msg.short_unique_id = bootloader_get_short_unique_id();
        allocation_msg.node_id = 0;

        /* Prepare the DynamicNodeIDAllocation request frame */
        tx_frame_metadata.transfer_id = 0u;
        tx_frame_metadata.last_frame = 1u;
        tx_frame_metadata.frame_index = 0u;
        tx_frame_metadata.source_node_id = 0u;
        tx_frame_metadata.transfer_type = MESSAGE_BROADCAST;
        tx_frame_metadata.data_type_id = UAVCAN_DYNAMICNODEIDALLOCATION_DTID;

        tx_frame_len = uavcan_pack_dynamicnodeidallocation(
            tx_frame_payload, &allocation_msg);

        /*
        This is repeated indefinitely, rate limited at 0.5 message per second
        and will exit to application if Timeout(Tboot) occurs.

        A Timeout(Tboot) will never occur if !AppValid
        */
        uint32_t last_request_cyccnt;
        do {
            /* UAVCANBootloader_v0.3 #18.7: Req559.DynamicNodeIDAllocation.uavcan(uuid,0) */
            tx_frame_metadata.transfer_id++;
            tx_frame_id = uavcan_make_frame_id(&tx_frame_metadata);
            uavcan_tx(tx_frame_id, tx_frame_len, tx_frame_payload);

            /*
            FIXME: Rate limit is set to 2 messages per second. A rate limit
            of 0.5 messages per second would mean only one message is sent
            during the default Tboot.
            */
            last_request_cyccnt = DWT->CYCCNT;
            while (!g_bootloader_tboot_timeout &&
                    !bootloader_timeout(last_request_cyccnt, 500u)) {
                got_frame = uavcan_rx(&rx_frame_id, &rx_frame_len,
                                      rx_frame_payload);
                if (got_frame) {
                    uavcan_parse_frame_id(&rx_frame_metadata, rx_frame_id);
                    if (rx_frame_metadata.data_type_id ==
                            UAVCAN_DYNAMICNODEIDALLOCATION_DTID) {
                        uavcan_unpack_dynamicnodeidallocation(
                            &allocation_response, rx_frame_payload);
                        if (allocation_response.short_unique_id ==
                                allocation_msg.short_unique_id) {
                            g_bootloader_node_id =
                                allocation_response.node_id;
                            break;
                        }
                    }
                }
            }

            /* UAVCANBootloader_v0.3 #20: [Timeout(Tboot)]: jump_to_app (node_id is 0 => App need to Allocate or use Static */
            if (g_bootloader_tboot_timeout) {
                goto boot;
            }
        } while (g_bootloader_node_id == 0u);

        /* TODO UAVCANBootloader_v0.3 #19.1: SetNodeID(newNodeID), Update(BootLoaderAppCommon, BL) */

        /* TODO UAVCANBootloader_v0.3 #19.2: INDICATE_ALLOCATED */
    }
    /* UAVCANBootloader_v0.3 #21: RestartFromApp */

    /* TODO UAVCANBootloader_v0.3 #21.1: initSeconds(0) */

    /* UAVCANBootloader_v0.3 #21.2: 550.NodeStatus.uavcan(uptime=0, STATUS_INITIALIZING) */
    g_bootloader_uptime = 0;
    bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                UAVCAN_NODESTATUS_STATUS_INITIALIZING);

    /* TODO UAVCANBootloader_v0.3 #21.2.1: Req551.GetNodeInfo.uavcan() */


    /* TODO UAVCANBootloader_v0.3 #21.2.1.1: [(AppBLRequest || WaitForGetNodeInfo) && AppValid)]: StartTboot([-DOPT_TBOOT) */
    /* TODO UAVCANBootloader_v0.3 #21.2.1.2: [AppValid]:SetAppVersion(SwVerion) */
    /* TODO UAVCANBootloader_v0.3 #21.2.1.3: 551.GetNodeInfo.uavcan */
    /* TODO UAVCANBootloader_v0.3 #21.2.1.4: [Timeout(Tboot)]: jump_to_app */

    /* TODO UAVCANBootloader_v0.3 #22: Req580.BeginFirmwareUpdate.uavcan */
    /* TODO UAVCANBootloader_v0.3 #22.1: BootLoadInProcess = true */

    /* TODO UAVCANBootloader_v0.3 #22.2: Resp580.BeginFirmwareUpdate.uavcan */
    /* TODO UAVCANBootloader_v0.3 #22.3: fwPath = image_file_remote_path */
    /* TODO UAVCANBootloader_v0.3 #22.4: fwSourceNodeID = source_node_id */
    /* TODO UAVCANBootloader_v0.3 #23: INDICATE_FW_UPDATE_START */
    /* UAVCANBootloader_v0.3 #24: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING) */
    bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                UAVCAN_NODESTATUS_STATUS_INITIALIZING);

    /* TODO UAVCANBootloader_v0.3 #25: SetRetryAndTimeout(3,1000MS) */
    /* TODO UAVCANBootloader_v0.3 #26: 585.GetInfo.uavcan(path) */
    /* TODO UAVCANBootloader_v0.3 #28: 585.GetInfo.uavcan(fw_path,fw_crc,fw_size...), */
    /* TODO UAVCANBootloader_v0.3 #27: validateFileInfo(file_info, &errorcode) */
    size_t fw_image_size;
    uint64_t fw_image_crc;
    fw_image_size = 0;
    fw_image_crc = 0;

    /* TODO UAVCANBootloader_v0.3 #28.1: [Retries==0]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
    /* FIXME UAVCANBootloader_v0.3 #28.2: [!validateFileInfo(file_info)]:1023.LogMessage.uavcan (errorcode) */
    if (0) {
        bootloader_send_log_message(g_bootloader_node_id,
                                    UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                    UAVCAN_LOGMESSAGE_STAGE_GET_INFO,
                                    UAVCAN_LOGMESSAGE_RESULT_FAIL);
        goto failure;
    }

    /* TODO UAVCANBootloader_v0.3 #28.4: save(file_info) */

    /* UAVCANBootloader_v0.3 #28.5: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING) */
    bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                UAVCAN_NODESTATUS_STATUS_INITIALIZING);

    /* UAVCANBootloader_v0.3 #28.6: 1023.LogMessage.uavcan("Erase") */
    bootloader_send_log_message(g_bootloader_node_id,
                                UAVCAN_LOGMESSAGE_LEVEL_INFO,
                                UAVCAN_LOGMESSAGE_STAGE_ERASE,
                                UAVCAN_LOGMESSAGE_RESULT_START);

    /* UAVCANBootloader_v0.3 #28.7: EraseFlash() */
    flash_error_t status;
    status = flash_erase();
    if (status != FLASH_OK) {
        /* UAVCANBootloader_v0.3 #28.9: [Erase Fail]:1023.LogMessage.uavcan */
        bootloader_send_log_message(g_bootloader_node_id,
                                    UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                    UAVCAN_LOGMESSAGE_STAGE_ERASE,
                                    UAVCAN_LOGMESSAGE_RESULT_FAIL);
        /* UAVCANBootloader_v0.3 #28.10: [Erase Fail]:550.NodeStatus.uavcan(uptime=t,STATUS_CRITICAL) */
        bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                    UAVCAN_NODESTATUS_STATUS_CRITICAL);

        /* TODO UAVCANBootloader_v0.3 #28.8: [Erase Failed]:INDICATE_FW_UPDATE_ERASE_FAIL */

        goto failure;
    }

    while (1) {
        retries = 3;
        last_request_cyccnt = DWT->CYCCNT;
        /* TODO UAVCANBootloader_v0.3 #28.11: SetRetryAndTimeout(3,1000MS) */

        /* TODO UAVCANBootloader_v0.3 #28.12: 588.Read.uavcan(0, path) */
        /* TODO UAVCANBootloader_v0.3 #33: 588.Read.uavcan(N,path) */

        while (!bootloader_timeout(last_request_cyccnt, 1000u)) {
            /* TODO UAVCANBootloader_v0.3 #28.12.1: 588.Read.uavcan(0,path,data) */
            /* TODO UAVCANBootloader_v0.3 #33.1: 583.Read.uavcan(0,path,data) */
            /* TODO UAVCANBootloader_v0.3 #34: ValidateReadResp(resp) */
            /* TODO UAVCANBootloader_v0.3 #30: SaveWord0 */

            if (0) { /* FIXME: if word program fail */
                /* UAVCANBootloader_v0.3 #31: [Program Fail]::1023.LogMessage.uavcan */
                bootloader_send_log_message(g_bootloader_node_id,
                                            UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                            UAVCAN_LOGMESSAGE_STAGE_PROGRAM,
                                            UAVCAN_LOGMESSAGE_RESULT_FAIL);

                /* UAVCANBootloader_v0.3 #32: [Program Fail]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
                bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                            UAVCAN_NODESTATUS_STATUS_CRITICAL);

                goto failure;
            }
        }

        if (retries) {
            /* UAVCANBootloader_v0.3 #36: [(retries != 0 && timeout) || !ValidReadReadResponse]:1023.LogMessage.uavcan */
            bootloader_send_log_message(g_bootloader_node_id,
                                        UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                        UAVCAN_LOGMESSAGE_STAGE_PROGRAM,
                                        UAVCAN_LOGMESSAGE_RESULT_FAIL);

            /* TODO UAVCANBootloader_v0.3 #35: [(retries != 0 && timeout) || !ValidReadReadResponse]:INDICATE_FW_UPDATE_INVALID_RESPONSE */

            retries--;
        } else {
            /* UAVCANBootloader_v0.3 #38: [(retries == 0 && timeout)]:1023.LogMessage.uavcan */
            bootloader_send_log_message(g_bootloader_node_id,
                                        UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                        UAVCAN_LOGMESSAGE_STAGE_PROGRAM,
                                        UAVCAN_LOGMESSAGE_RESULT_FAIL);

            /* TODO UAVCANBootloader_v0.3 #37: [(retries == 0 && timeout]:INDICATE_FW_UPDATE_TIMEOUT */

            /* UAVCANBootloader_v0.3 #40: [Retries==0]:InvalidateBootLoaderAppCommon(), RestartWithDelay(20,000ms) */
            goto failure;
        }
    }

    /* UAVCANBootloader_v0.3 #41: CalulateCRC(file_info) */
    uint64_t flash_image_crc;
    flash_image_crc = flash_crc(FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET,
                                fw_image_size);
    if (flash_image_crc != fw_image_crc) {
        /* UAVCANBootloader_v0.3 #42: [crc Faill]:1023.LogMessage.uavcan */
        bootloader_send_log_message(g_bootloader_node_id,
                                    UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                    UAVCAN_LOGMESSAGE_STAGE_VALIDATE,
                                    UAVCAN_LOGMESSAGE_RESULT_FAIL);

        /* TODO UAVCANBootloader_v0.3 #43: [crc Fail]:INDICATE_FW_UPDATE_INVALID_CRC */

        /* UAVCANBootloader_v0.3 #45: [crc Fail]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
        goto failure;
    }

    /* TODO UAVCANBootloader_v0.3 #46: [crc == fw_crc]:write word0 */
    /* TODO UAVCANBootloader_v0.3 #47: ValidateBootLoaderAppCommon() */

    /* TODO UAVCANBootloader_v0.3 #48: KickTheDog() */

    /* UAVCANBootloader_v0.3 #49: 550.NodeStatus.uavcan(uptime=t, STATUS_INITIALIZING) */
    bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                UAVCAN_NODESTATUS_STATUS_INITIALIZING);

    /* TODO UAVCANBootloader_v0.3 #50: jump_to_app */


    uint32_t bytes[8];
    uint32_t id;
    size_t len;
    uavcan_rx(&id, &len, bytes);

    flash_write_data(FLASH_BASE, 8, (uint8_t*)bytes);

    while (1);
    bootloader_reset();

boot:
    while(1);

failure:
    /* TODO: stop SysTick */
    /* UAVCANBootloader_v0.3 #28.3: [!validateFileInfo(file_info)]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #39: [(retries == 0 && timeout)]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    /* UAVCANBootloader_v0.3 #44: [crc Fail]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
    bootloader_send_node_status(g_bootloader_node_id, g_bootloader_uptime,
                                UAVCAN_NODESTATUS_STATUS_CRITICAL);
    while(1);
}


void bootloader_gpio_init() {
    /*
    TODO: replace this with e.g. NuttX init code and compile-time options.

    We need to enable the CAN RX and TX pins, do whatever we need to take the
    CAN transceiver out of silent (if supported by the hardware), and ensure
    hardware is left in a safe state.
    */
    uint32_t offset;

    /* GPIO_SetBits(PORT_CAN_SILENT, PIN_CAN_SILENT.GPIO_Pin); */
    PORT_CAN_SILENT->BSRR = (1u << PIN_CAN_SILENT_INDEX);

    /*
    Configure RX and TX pins with:
    GPIO_Mode_AF
    GPIO_Speed_Level_2
    GPIO_OType_PP
    GPIO_PuPd_NOPULL

    Configure PIN_CAN_SILENT as above but GPIO_Mode_OUT.
    */

    /* GPIO_Init(PORT_CAN_TX, PIN_CAN_TX); */
    offset = PIN_CAN_TX_INDEX * 2u;
    PORT_CAN_TX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_TX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_TX->OTYPER = (uint16_t)
        (PORT_CAN_TX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_TX_INDEX));
    PORT_CAN_TX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_TX_INDEX);
    PORT_CAN_TX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_TX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_TX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_TX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_RX, PIN_CAN_RX); */
    offset = PIN_CAN_RX_INDEX * 2u;
    PORT_CAN_RX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_RX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_RX->OTYPER = (uint16_t)
        (PORT_CAN_RX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_RX_INDEX));
    PORT_CAN_RX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_RX_INDEX);
    PORT_CAN_RX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_RX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_RX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_RX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_SILENT, PIN_CAN_SILENT); */
    offset = PIN_CAN_SILENT_INDEX * 2u;
    PORT_CAN_SILENT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_SILENT->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_SILENT->OTYPER = (uint16_t)
        (PORT_CAN_SILENT->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_SILENT_INDEX));
    PORT_CAN_SILENT->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_SILENT_INDEX);
    PORT_CAN_SILENT->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_SILENT->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_SILENT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_SILENT->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_PinAFConfig(PORT_CAN_RX, GPIO_PinSource11, GPIO_AF_9); */
    offset = (PIN_CAN_RX_INDEX & 7u) * 4u;
    PORT_CAN_RX->AFR[PIN_CAN_RX_INDEX >> 3u] &= ~(0xFu << offset);
    PORT_CAN_RX->AFR[PIN_CAN_RX_INDEX >> 3u] |= 0x9u << offset;

    /* GPIO_PinAFConfig(PORT_CAN_TX, GPIO_PinSource12, GPIO_AF_9); */
    offset = (PIN_CAN_TX_INDEX & 7u) * 4u;
    PORT_CAN_TX->AFR[PIN_CAN_TX_INDEX >> 3u] &= ~(0xFu << offset);
    PORT_CAN_TX->AFR[PIN_CAN_TX_INDEX >> 3u] |= 0x9u << offset;
}


void bootloader_reset(void) {
    __DSB();
    SCB->AIRCR = ((0x5FAu << SCB_AIRCR_VECTKEY_Pos) |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    for (volatile uint8_t cond = 1; cond;);
}


uint64_t bootloader_get_short_unique_id(void) {
    /* FIXME */
    return 0x1122334455667788L & 0x01FFFFFFFFFFFFFFL;
}


void bootloader_send_node_status(
    uint8_t node_id,
    uint32_t uptime_sec,
    uint8_t status_code
) {
    uavcan_nodestatus_t message;
    uavcan_frame_id_t metadata;
    uint32_t frame_id, payload[2];
    size_t frame_len;

    metadata.transfer_id = 0;
    metadata.last_frame = 1u;
    metadata.frame_index = 0;
    metadata.source_node_id = node_id;
    metadata.transfer_type = MESSAGE_BROADCAST;
    metadata.data_type_id = UAVCAN_NODESTATUS_DTID;
    frame_id = uavcan_make_frame_id(&metadata);

    message.uptime_sec = uptime_sec;
    message.status_code = status_code;
    frame_len = uavcan_pack_nodestatus(payload, &message);
    uavcan_tx(frame_id, frame_len, payload);
}


void bootloader_send_log_message(
    uint8_t node_id,
    uint8_t level,
    uint8_t stage,
    uint8_t status
) {
    uavcan_logmessage_t message;
    uavcan_frame_id_t metadata;
    uint32_t frame_id, payload[2];
    size_t frame_len;

    metadata.transfer_id = 0;
    metadata.last_frame = 1u;
    metadata.frame_index = 0;
    metadata.source_node_id = node_id;
    metadata.transfer_type = MESSAGE_BROADCAST;
    metadata.data_type_id = UAVCAN_LOGMESSAGE_DTID;
    frame_id = uavcan_make_frame_id(&metadata);

    message.level = level;
    message.message[0] = stage;
    message.message[1] = status;
    frame_len = uavcan_pack_logmessage(payload, &message);
    uavcan_tx(frame_id, frame_len, payload);
}


uint8_t bootloader_timeout(uint32_t start_cyccnt, uint32_t millis) {
    if (start_cyccnt > 0) {
        return (DWT->CYCCNT - start_cyccnt) / (OPT_CPU_FREQ_HZ / 1000u) >= millis;
    } else {
        return 0;
    }
}
