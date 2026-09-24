/**
 * @file flash_defs.hpp
 * @brief Bad Page Table Implementation
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include <Arduino.h>

#include "bad_page_table.h"
#include "flash.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"

#include "CommsSerial.h"
#include <stdlib.h>
#include <string.h>

extern uint32_t _bpt_start;

#define STM32_FLASH_WORD_SIZE 32u 
#define BPT_MAGIC_NUMBER 0x0123ABCDu 

uint32_t *bpt_magic = reinterpret_cast<uint32_t *>(&_bpt_start);
uint8_t *bpt_table = reinterpret_cast<uint8_t *>(&_bpt_start) + sizeof(BPT_MAGIC_NUMBER);

namespace Bad_Page_Table {

bool begin(void) {
    if (*bpt_magic == BPT_MAGIC_NUMBER) {
        return true;
    }

    const int key_min = 1000;
    const int key_max = 10000;
    srand(millis());
    int key = rand() % (key_max - key_min) + key_min;

    CommsSerial.println("WARNING: Bad Page Table not initialized - a factory bad-block scan is required.");
    CommsSerial.printf("Enter %d to proceed: \n", key);

    char *res = CommsSerial.readline();
    if (atoi(res) != key) {
        CommsSerial.println("Incorrect key entered.");
        return false;
    }

    // Reset the whole region to a known-erased (0xFF = all good) state before scanning, in
    // case it was left in an unknown state (e.g. a corrupted magic number from a previous
    // partial/failed init). mark_page_bad() only ever clears bits from then on, which never
    // needs another erase.
    FLASH_EraseInitTypeDef erase_init{};
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_2;
    erase_init.Sector = 0;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error;
    HAL_FLASH_Unlock();
    bool erase_ok = HAL_FLASHEx_Erase(&erase_init, &sector_error) == HAL_OK;
    HAL_FLASH_Lock();

    if (!erase_ok) {
        CommsSerial.println("Bad Page Table erase failed.");
        return false;
    }

    CommsSerial.println("Scanning for factory bad blocks. This may take a while.");

    if (!scan_factory_bad_blocks()) {
        CommsSerial.println("Factory bad-block scan failed.");
        return false;
    }

    // Write the magic number last, so an interrupted/failed scan never leaves a table that
    // looks initialized but isn't actually complete.
    uint8_t word_buf[STM32_FLASH_WORD_SIZE];
    memcpy(word_buf, bpt_magic, STM32_FLASH_WORD_SIZE);

    uint32_t magic = BPT_MAGIC_NUMBER;
    memcpy(word_buf, &magic, sizeof(magic));

    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, reinterpret_cast<uint32_t>(bpt_magic), reinterpret_cast<uint32_t>(word_buf));
    HAL_FLASH_Lock();

    CommsSerial.println("Bad Page Table initialized.");
    return true;
};

bool scan_factory_bad_blocks(void) {
    for (uint32_t i = 0; i < NAND_NUM_BLOCKS; i++) {
        uint32_t cur_page = i * NAND_PAGES_PER_BLOCK;

        uint8_t bad_block_mark;
        if (Flash::read_spare(cur_page, &bad_block_mark, 1) != FLASH_SUCCESS) {
            return false;
        }

        if (bad_block_mark != 0xFF) {
            mark_page_bad(cur_page);
        }
    }

    return true;
};

static uint32_t get_bpt_bad_byte(uint32_t block_num) {
    return block_num / 8;
}

static uint8_t get_bpt_bad_bit(uint32_t block_num) {
    return (1 << (block_num % 8));
}

bool is_page_good(uint32_t page_address) {
    uint32_t block_num = page_address / NAND_PAGES_PER_BLOCK;
    uint32_t byte = get_bpt_bad_byte(block_num);
    uint8_t bit = get_bpt_bad_bit(block_num);
    uint8_t *bad_page_byte = bpt_table + byte;

    if ((*bad_page_byte & bit) == 0) {
        return false;
    } else {
        return true;
    }
    
};

void mark_page_bad(uint32_t page_address) {
    uint32_t block_num = page_address / NAND_PAGES_PER_BLOCK;
    uint32_t byte = get_bpt_bad_byte(block_num);
    uint8_t bit = ~(get_bpt_bad_bit(block_num));

    // flash can only be programmed in 32 byte aligned chunks
    uint32_t byte_addr = reinterpret_cast<uint32_t>(bpt_table + byte);
    uint32_t word_addr = byte_addr - (byte_addr % STM32_FLASH_WORD_SIZE);
    uint32_t byte_offset_in_word = byte_addr - word_addr;

    uint8_t word_buf[STM32_FLASH_WORD_SIZE];
    memcpy(word_buf, reinterpret_cast<uint8_t *>(word_addr), STM32_FLASH_WORD_SIZE);
    word_buf[byte_offset_in_word] &= bit;

    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, word_addr, reinterpret_cast<uint32_t>(word_buf));
    HAL_FLASH_Lock();
};

}