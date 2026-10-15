/**
 * @file  bad_page_table.cpp
 * @brief Bad Page Table Implementation for STM32H7 internal flash
 * @note  There are effectively two bad page tables, one is the initial
 *        table filled only at init and the other is the entries table, filled
 *        dynamically over time.
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
#define BPT_EMPTY_ENTRY 0xFFFFFFFFu

constexpr uint32_t BPT_MAGIC_NUMBER = 0x0123ABCDu;
constexpr uint32_t BPT_MAX_ENTRIES =
    (FLASH_SECTOR_SIZE - STM32_FLASH_WORD_SIZE - NUM_BYTES_TRACKING_BLOCKS) / STM32_FLASH_WORD_SIZE;

uint32_t *bpt_magic = &_bpt_start;
uint8_t *bpt_table = reinterpret_cast<uint8_t *>(&_bpt_start) + STM32_FLASH_WORD_SIZE;
uint8_t *bpt_entries = bpt_table + NUM_BYTES_TRACKING_BLOCKS;

namespace Bad_Page_Table {

namespace {

bool program_flash_word(uint32_t flash_addr, uint32_t word_addr) {
  HAL_StatusTypeDef status;

  HAL_FLASH_Unlock();
  // Flash Typeprogram flashword means program 1 flash word, 32 bytes on the H7
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_addr, word_addr);
  HAL_FLASH_Lock();

  return status == HAL_OK;
}

uint32_t get_entry_addr(uint32_t index) {
  return reinterpret_cast<uint32_t>(bpt_entries + index * STM32_FLASH_WORD_SIZE);
}

uint32_t get_entry(uint32_t index) {
  return *reinterpret_cast<uint32_t *>(get_entry_addr(index));
}

uint32_t get_bpt_bad_byte(uint32_t block_num) {
  return block_num / 8;
}

uint8_t get_bpt_bad_bit(uint32_t block_num) {
  return (1 << (block_num % 8));
}

int generate_key() {
  const int key_min = 1000;
  const int key_max = 10000;
  srand(millis());
  return (rand() % (key_max - key_min) + key_min); 
}

} // namespace

bool begin(void) {
  if (*bpt_magic == BPT_MAGIC_NUMBER) {
    return true;
  }

  int key = generate_key();

  CommsSerial.println("WARNING: Bad Page Table not initialized - a factory bad-block scan is required.");
  CommsSerial.printf("Enter %d to proceed: \n", key);

  char *res = CommsSerial.readline();
  if (atoi(res) != key) {
    CommsSerial.println("Incorrect key entered.");
    return false;
  }

  // erase bad page table
  FLASH_EraseInitTypeDef erase_init = {
    .TypeErase = FLASH_TYPEERASE_SECTORS,
    .Banks = FLASH_BANK_2,
    .Sector = FLASH_SECTOR_7,
    .NbSectors = 1,
    .VoltageRange = FLASH_VOLTAGE_RANGE_3
  };
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

  uint8_t word_buf[STM32_FLASH_WORD_SIZE];
  memset(word_buf, 0, sizeof(word_buf));
  memcpy(word_buf, &BPT_MAGIC_NUMBER, sizeof(BPT_MAGIC_NUMBER));

  uint32_t flash_magic_addr = reinterpret_cast<uint32_t>(bpt_magic);
  uint32_t word_buf_addr = reinterpret_cast<uint32_t>(word_buf);

  if (!program_flash_word(flash_magic_addr, word_buf_addr)) {
    CommsSerial.println("Flash program failed");
    return false;
  }

  CommsSerial.println("Bad Page Table initialized.");
  return true;
};

bool scan_factory_bad_blocks(void) {
  uint8_t table_buf[NUM_BYTES_TRACKING_BLOCKS];
  memset(table_buf, PAGE_MARKED_GOOD, sizeof(table_buf));

  for (uint32_t i = 0; i < NAND_NUM_BLOCKS; i++) {
    uint32_t cur_page = i * NAND_PAGES_PER_BLOCK;

    uint8_t bad_block_mark;
    if (Flash::read_spare(cur_page, 0, &bad_block_mark, 1, false) != FLASH_SUCCESS) {
      return false;
    }

    if (bad_block_mark != PAGE_MARKED_GOOD) {
      table_buf[get_bpt_bad_byte(i)] &= ~get_bpt_bad_bit(i);
    }
  }

  // flash can only be programmed once per erase, in 32 byte aligned chunks
  for (uint32_t offset = 0; offset < NUM_BYTES_TRACKING_BLOCKS; offset += STM32_FLASH_WORD_SIZE) {
    uint32_t flash_addr = reinterpret_cast<uint32_t>(bpt_table + offset);
    uint32_t table_addr = reinterpret_cast<uint32_t>(table_buf + offset);
    if (!program_flash_word(flash_addr, table_addr)) {
      return false;
    }
  }

  return true;
};

bool is_page_good(uint32_t page_address) {
  if (page_address >= NAND_NUM_PAGES) {
    return false;
  }

  uint32_t block_num = page_address / NAND_PAGES_PER_BLOCK;
  uint32_t byte = get_bpt_bad_byte(block_num);
  uint8_t bit = get_bpt_bad_bit(block_num);
  uint8_t *bad_page_byte = bpt_table + byte;

  if (!(*bad_page_byte & bit)) {
    return false;
  }

  for (uint32_t i = 0; i < BPT_MAX_ENTRIES; i++) {
    uint32_t entry = get_entry(i);
    if (entry == BPT_EMPTY_ENTRY) {
      break;
    }

    if (entry == block_num) {
      return false;
    }
  }

  return true;
};

bool mark_page_bad(uint32_t page_address) {
  if (page_address >= NAND_NUM_PAGES) {
    return false;
  }

  if (!is_page_good(page_address)) {
    return true;
  }

  uint32_t block_num = page_address / NAND_PAGES_PER_BLOCK;

  for (uint32_t i = 0; i < BPT_MAX_ENTRIES; i++) {
    if (get_entry(i) != BPT_EMPTY_ENTRY) {
      continue;
    }

    uint8_t word_buf[STM32_FLASH_WORD_SIZE];
    memset(word_buf, PAGE_MARKED_GOOD, sizeof(word_buf));
    memcpy(word_buf, &block_num, sizeof(block_num));

    uint32_t word_buf_addr = reinterpret_cast<uint32_t>(word_buf);
    uint32_t entry_addr = get_entry_addr(i);

    return program_flash_word(entry_addr, word_buf_addr);
  }

  return false;
};

} // namespace Bad_Page_Table