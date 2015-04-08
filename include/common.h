#pragma once


/*
Application firmware descriptor. This goes somewhere within the first several
kilobytes of the application; the bootloader scans through the application
image until it finds the signature bytes. The descriptor must be aligned on an
8-byte boundary.

The CRC is calculated over all fields as well as the firmware itself,
although for the purposes of CRC calculation the image_crc field is set to 0.
The length is the length of the entire image.
*/
typedef struct {
    uint64_t signature;
    uint64_t image_crc;
    uint32_t image_size;
    uint32_t vcs_commit;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t reserved[6];
} __attribute__((packed)) app_descriptor_t;


#define APP_DESCRIPTOR_SIGNATURE_LO 0x65445041u /* eDPA */
#define APP_DESCRIPTOR_SIGNATURE_HI 0x30306373u /* 00cs */


/*
Bootloader/app common structure.

This is loaded from the CAN filter registers on reset, and is written back to
the CAN filter registers prior to application launch.

The application must write BOOTLOADER_COMMON_APP_SIGNATURE to the signature
field when passing data to the bootloader; when the bootloader passes data
to the app, it must write BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE to the
signature field.

The CRC is calculated over F1R1, F1R2, F2R1 and F2R2.
*/
typedef struct {
    uint64_t crc;          /* To/from CAN1->F3R1, CAN1->F3R2 */
    uint32_t signature;    /* To/from CAN1->F1R1 */
    uint32_t bus_speed;    /* To/from CAN1->F1R2 */
    uint32_t node_id;      /* To/from CAN1->F2R1 */
    uint32_t uptime;       /* To CAN1->F2R2*/
    uint32_t app_progress; /* From CAN1->F2R2 */
} __attribute__((packed)) bootloader_app_common_t;


#define BOOTLOADER_COMMON_APP_SIGNATURE 0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE 0xB0A0424Cu


uint8_t common_is_valid(void);
void common_read(bootloader_app_common_t *common);
void common_write(bootloader_app_common_t *common);
void common_invalidate(void);
