/*
 * CANopen Object Dictionary storage object (blank example).
 *
 * @file        CO_storageBlank.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "CO_storageFlash.h"
#include "301/crc16-ccitt.h"
#include "gd32f10x_fmc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE


#define PARAM_STORE_PASSWORD 0x65766173
#define PARAM_RESTORE_PASSWORD 0x64616F6C

#define DEBUG 0

#define LAST_PAGE_ADDRESS 0x08008000

#define PAGES_PER_FLASH_AREA 1
#define FLASH_PAGE_SIZE 0x400
#define CO_OD_FLASH_PARAM_RUNTIME                                              \
    LAST_PAGE_ADDRESS - (4 * PAGES_PER_FLASH_AREA * FLASH_PAGE_SIZE)

static uint32_t flash_page_num = CO_OD_FLASH_PARAM_RUNTIME;

#define CO_UNUSED(v) (void)(v)

static void fmc_program(uint32_t start_addr, uint32_t end_addr, uint32_t *data)
{
    uint32_t addressPtr = 0;
    fmc_unlock();

    /* program flash */
    for (addressPtr = start_addr; addressPtr < end_addr; addressPtr += 4) {
        fmc_word_program(addressPtr, *data);
        data = data + 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the program operation */
    fmc_lock();

    // {
    //     uint32_t *ptrd;
    //     uint32_t i;

    //     ptrd = (uint32_t *)start_addr;

    //     /* check flash whether has been programmed */
    // }
}

size_t CO_flash_getPageaddr(size_t len, uint8_t *page, bool_t *overflow)
{
    size_t addr;
    uint8_t pages =
        (len / FLASH_PAGE_SIZE) + ((len % FLASH_PAGE_SIZE == 0) ? 0 : 1);
    *page = pages;
    addr = flash_page_num;
    flash_page_num = flash_page_num + pages * FLASH_PAGE_SIZE;
    if (flash_page_num > LAST_PAGE_ADDRESS) {
        *overflow = true;
    }

    return addr;
}


static void fmc_erase_pages(uint32_t start_addr, uint8_t page)
{
    uint32_t erase_counter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for (erase_counter = 0; erase_counter < page; erase_counter++) {
        fmc_page_erase(start_addr + (FLASH_PAGE_SIZE * erase_counter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

static void fmc_read(uint32_t flash_address, void *ram_address, size_t len)
{
    uint8_t *p_flash = (uint8_t *)flash_address;
    uint8_t *p_ram = (uint8_t *)ram_address;
    uint32_t idx;

    for (idx = 0; idx < len; idx++) {
        *p_ram = *(__IO uint8_t *)p_flash;
        p_ram++;
        p_flash++;
    }
}

/*
 * Function for writing data on "Store parameters" command - OD object 1010
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t storeFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
    ODR_t ret = ODR_OK;
    uint32_t addressFinal = entry->flashPageAddr + entry->len;

    if (entry->len > (entry->page * FLASH_PAGE_SIZE)) {
        return ODR_OUT_OF_MEM;
    }

    CO_LOCK_OD(CANmodule);
    fmc_erase_pages(entry->flashPageAddr, entry->page);

    // 写数据到CO_OD_FLASH_PARAM_RUNTIME
    fmc_program(entry->flashPageAddr, addressFinal, entry->addr);
    CO_UNLOCK_OD(CANmodule);

    // 写crc
    uint32_t crc_store = crc16_ccitt(entry->addr, entry->len, 0);
    fmc_program(addressFinal, addressFinal + sizeof(crc_store), &crc_store);

    /* Verify data */
    uint8_t *buf = NULL;
    size_t cnt = 0;
    uint32_t crc_verify, crc_read;
    buf = malloc(entry->len + 4);

    if (buf != NULL) {

        fmc_read(entry->flashPageAddr, buf, entry->len + 4);
        crc_verify = crc16_ccitt(buf, entry->len, 0);
        memcpy(&crc_read, &buf[entry->len], sizeof(crc_read));
    }

    /* If size or CRC differs, report error */
    if (buf == NULL || crc_store != crc_verify || crc_store != crc_read) {
        ret = ODR_HW;
    }


    return ret;
}


/*
 * Function for restoring data on "Restore default parameters" command - OD
 * 1011
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t restoreFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
    /* disable (delete) the file, so default values will stay after startup
     */
    (void)CANmodule;
    ODR_t ret = ODR_OK;

    if (entry->len > (entry->page * FLASH_PAGE_SIZE)) {
        return ODR_OUT_OF_MEM;
    }

    uint32_t addressFinal = entry->flashPageAddr + 1;
    uint32_t buf = 0x55555555;

    CO_LOCK_OD(CANmodule);
    fmc_erase_pages(entry->flashPageAddr, entry->page);

    // 写数据到CO_OD_FLASH_PARAM_RUNTIME
    fmc_program(entry->flashPageAddr, addressFinal, &buf);
    CO_UNLOCK_OD(CANmodule);

    return ret;
}


CO_ReturnError_t CO_storageFlash_init(CO_storage_t *storage,
                                      CO_CANmodule_t *CANmodule,
                                      OD_entry_t *OD_1010_StoreParameters,
                                      OD_entry_t *OD_1011_RestoreDefaultParam,
                                      CO_storage_entry_t *entries,
                                      uint8_t entriesCount,
                                      uint32_t *storageInitError)
{
    CO_ReturnError_t ret;
    bool_t flashOvf = false;

    /* verify arguments */
    if (storage == NULL || entries == NULL || entriesCount == 0 ||
        storageInitError == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    storage->enabled = false;

    /* initialize storage and OD extensions */
    ret = CO_storage_init(storage,
                          CANmodule,
                          OD_1010_StoreParameters,
                          OD_1011_RestoreDefaultParam,
                          storeFlash,
                          restoreFlash,
                          entries,
                          entriesCount);
    if (ret != CO_ERROR_NO) {
        return ret;
    }

    /* initialize entries */
    *storageInitError = 0;
    for (uint8_t i = 0; i < entriesCount; i++) {
        CO_storage_entry_t *entry = &entries[i];
        bool_t isAuto = (entry->attr & CO_storage_auto) != 0;

        /* verify arguments */
        if (entry->addr == NULL || entry->len == 0 || entry->subIndexOD < 2) {
            *storageInitError = i;
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }

        uint8_t page = 0;

        entry->flashPageAddr =
            CO_flash_getPageaddr(entry->len, &page, &flashOvf);
        entry->page = page;

        /* verify if flash is too small */
        if (flashOvf) {
            *storageInitError = i;
            return CO_ERROR_OUT_OF_MEMORY;
        }

        uint8_t *buf = NULL;
        buf = malloc(entry->len + 4);
        if (buf == NULL) {
            *storageInitError = i;
            return CO_ERROR_OUT_OF_MEMORY;
        }

        /* Read data into temporary buffer first. Then verify and copy to addr*/
        if (buf != NULL) {

            fmc_read(entry->flashPageAddr, buf, entry->len + 4);
            /* If flash is empty, just skip loading, default values will be
             * used, no error. Otherwise verify length and crc and copy data. */
            if (!(buf[0] == 0x55 && buf[2] == 0x55)) {
                uint32_t crc1, crc2;
                crc1 = crc16_ccitt(buf, entry->len, 0);
                memcpy(&crc2, &buf[entry->len], sizeof(crc2));

                if (crc1 == crc2) {
                    memcpy(entry->addr, buf, entry->len);
                    entry->crc = crc1;
                } else {
                    ret = CO_ERROR_DATA_CORRUPT;
                }
            }
        }

        free(buf);
    }

    storage->enabled = true;
    return ret;
}

uint32_t CO_storageFlash_auto_process(CO_storage_t *storage, bool_t closeFiles)
{
    uint32_t storageError = 0;

    /* verify arguments */
    if (storage == NULL) {
        return false;
    }

    /* loop through entries */
    for (uint8_t i = 0; i < storage->entriesCount; i++) {
        CO_storage_entry_t *entry = &storage->entries[i];

        if ((entry->attr & CO_storage_auto) == 0)
            continue;

        /* If CRC of the current data differs, save the file */
        uint16_t crc = crc16_ccitt(entry->addr, entry->len, 0);
        if (crc != entry->crc) {
            CO_LOCK_OD(CANmodule);
            fmc_erase_pages(entry->flashPageAddr, entry->page);

            // 写数据到CO_OD_FLASH_PARAM_RUNTIME
            fmc_program(entry->flashPageAddr,
                        entry->flashPageAddr + entry->len,
                        entry->addr);
            CO_UNLOCK_OD(CANmodule);

            // 写crc
            uint32_t crc_store = crc16_ccitt(entry->addr, entry->len, 0);
            fmc_program(entry->flashPageAddr + entry->len,
                        entry->flashPageAddr + entry->len + sizeof(crc_store),
                        &crc_store);
        }

        if (closeFiles) {
        }
    }

    return storageError;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
