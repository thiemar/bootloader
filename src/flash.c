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

    /* Erase the whole application flash region */
    for (addr = APP_FLASH_ADDR; addr < APP_FLASH_END && status == FLASH_OK;
            addr += FLASH_PAGE_SIZE) {
        __disable_irq();

        /* FLASH_Unlock(); */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        /* FLASH_ErasePage(addr); */
        status = flash_wait_();
        if (status == FLASH_OK) {
            FLASH->CR |= FLASH_CR_PER;
            FLASH->AR  = addr;
            FLASH->CR |= FLASH_CR_STRT;

            status = flash_wait_();

            FLASH->CR &= ~FLASH_CR_PER;
        }

        /* FLASH_Lock(); */
        FLASH->CR |= FLASH_CR_LOCK;

        __enable_irq();
    }

    return status;
}


flash_error_t flash_write_word(
    uint32_t flash_address,
    const uint8_t data[4]
) {
    flash_error_t status;

    if (flash_address < APP_FLASH_ADDR ||
            flash_address + 4u > APP_FLASH_END) {
        return FLASH_ERROR;
    }

    __disable_irq();

    /* FLASH_Unlock(); */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* FLASH_ProgramWord(addr, word); */
    FLASH->CR |= FLASH_CR_PG;

    ((volatile uint16_t*)flash_address)[0] =
        (uint16_t)(data[0] | (data[1] << 8u));
    status = flash_wait_();

    if (status == FLASH_OK) {
        ((volatile uint16_t*)flash_address)[1] =
            (uint16_t)(data[2] | (data[3] << 8u));
        status = flash_wait_();
    }

    FLASH->CR &= ~FLASH_CR_PG;

    /* FLASH_Lock(); */
    FLASH->CR |= FLASH_CR_LOCK;

    __enable_irq();

    return status;
}
