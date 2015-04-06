#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "bootloader_config.h"
#include "flash.h"
#include "crc.h"


#define APP_FLASH_ADDR (FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET)
#define APP_FLASH_END (APP_FLASH_ADDR + OPT_APPLICATION_IMAGE_LENGTH)


static flash_error_t flash_wait_(void) {
    while (FLASH->SR & FLASH_SR_BSY);

    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR)) {
        return FLASH_ERROR;
    } else {
        return FLASH_OK;
    }
}


flash_error_t flash_erase(void) {
    /*
    FIXME (?): this may take a long time, and while flash is being erased it
    might not be possible to execute interrupts, send NodeStatus messages etc.

    It might be better to re-lock flash, enable interrupts and wait for a bit
    after each page.
    */

    uint32_t addr;
    flash_error_t status;

    status = FLASH_OK;

    /* TODO: disable interrupts */

    /* FLASH_Unlock(); */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* Erase the whole application flash region */
    for (addr = APP_FLASH_ADDR; addr < APP_FLASH_END; addr += FLASH_PAGE_SIZE) {
        /* FLASH_ErasePage(addr); */
        status = flash_wait_();
        if (status) {
            break;
        }

        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR  = addr;
        FLASH->CR |= FLASH_CR_STRT;

        status = flash_wait_();

        FLASH->CR &= ~FLASH_CR_PER;
        if (status) {
            break;
        }
    }

    /* FLASH_Lock(); */
    FLASH->CR |= FLASH_CR_LOCK;

    /* TODO: enable interrupts */

    return status;
}


flash_error_t flash_write_data(
    uint32_t flash_address,
    size_t length,
    const uint8_t* data
) {
    flash_error_t status;
    uint32_t start_address, end_address;

    status = FLASH_OK;
    start_address = flash_address;
    end_address = start_address + length;

    if (start_address < APP_FLASH_ADDR || end_address > APP_FLASH_END) {
        return FLASH_ERROR;
    }

    /* TODO: disable interrupts */

    /* FLASH_Unlock(); */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* FLASH_ProgramWord(addr, word); */
    FLASH->CR |= FLASH_CR_PG;
    for (; flash_address < end_address && status == FLASH_OK;
            flash_address += 2u) {
        *(volatile uint16_t*)flash_address = (uint16_t)
            ((uint16_t)data[flash_address - start_address] |
             (uint16_t)(data[1u + flash_address - start_address] << 8u));
        status = flash_wait_();
    }
    FLASH->CR &= ~FLASH_CR_PG;

    /* FLASH_Lock(); */
    FLASH->CR |= FLASH_CR_LOCK;

    /* TODO: enable interrupts */

    /* Loop ended early -- a word must have been written incorrectly */
    if (flash_address != end_address) {
        return FLASH_ERROR;
    } else {
        return FLASH_OK;
    }
}


uint64_t flash_crc(
    uint32_t flash_address,
    size_t length,
    uint64_t initial_crc
) {
    volatile uint8_t *flash_ptr;
    uint64_t crc;

    flash_ptr = (volatile uint8_t*)flash_address;
    crc = initial_crc;
    for (; (uint32_t)flash_ptr < flash_address + length; flash_ptr++) {
        crc = crc64_add(crc, *flash_ptr);
    }
    crc ^= CRC64_OUTPUT_XOR;

    return crc;
}
