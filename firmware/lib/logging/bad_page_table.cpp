/**
 * @file  bad_page_table.cpp
 * @brief Bad Page Table Implementation for STM32H7 internal flash
 * @note  There are effectively two bad page tables, one is the factory
 *        table filled only at init and the other is the entries table, filled
 *        dynamically over time.
 *
 *        Layout of the table's flash sector (see .bpt_table in ldscript.ld):
 *          flash word 0      magic number, set once the factory scan is done
 *          next 128 bytes    factory table, 1 bit per NAND block (1 = good, 0 = bad)
 *          remaining words   runtime entries, one per flash word, each holding
 *                            the row address of a page that failed to program
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include <Arduino.h>

#include "bad_page_table.h"
#include "flash.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"

#include "CommsSerial.h"
#include <string.h>

// The STM32H7 programs its flash one 32 byte flash word at a time,
// and a flash word can only be programmed once per sector erase
#define STM32_FLASH_WORD_SIZE 32u
#define STM32_FLASH_WORD_U32S (STM32_FLASH_WORD_SIZE / sizeof(uint32_t))

// Sector holding the table, must match BPT_FLASH in ldscript.ld
#define BPT_BANK          FLASH_BANK_2
#define BPT_SECTOR        FLASH_SECTOR_7
#define BPT_NUM_SECTORS   1u
#define BPT_VOLTAGE_RANGE FLASH_VOLTAGE_RANGE_3

#define BPT_MAGIC_NUMBER       0x0123ABCDu
#define BPT_EMPTY_ENTRY        0xFFFFFFFFu
#define BPT_FACTORY_TABLE_SIZE (NAND_NUM_BLOCKS / 8)

#define ERASED_BYTE          0xFFu
#define NAND_GOOD_BLOCK_MARK 0xFFu

static_assert(BPT_FACTORY_TABLE_SIZE % STM32_FLASH_WORD_SIZE == 0,
              "factory table must be a whole number of flash words");

extern uint32_t _bpt_start;

namespace Bad_Page_Table {

namespace {

// take the entire sector and exclude the word used for 
// the magic entry and exclude the bpt table, then divide by size of 
// words to get max number of pages we can dynamically mark as bad
constexpr uint32_t BPT_MAX_ENTRIES =
    (FLASH_SECTOR_SIZE - STM32_FLASH_WORD_SIZE - BPT_FACTORY_TABLE_SIZE) / STM32_FLASH_WORD_SIZE;

// Fixed addresses of each part of the table in STM32 flash
uint8_t *const bpt_start = reinterpret_cast<uint8_t *>(&_bpt_start);
uint32_t *const bpt_magic = reinterpret_cast<uint32_t *>(bpt_start);
uint8_t *const bpt_factory_table = bpt_start + STM32_FLASH_WORD_SIZE;
uint8_t *const bpt_entries = bpt_factory_table + BPT_FACTORY_TABLE_SIZE;

bool program_flash_word(uint32_t flash_addr, const uint32_t *data) {
  HAL_FLASH_Unlock();
  HAL_StatusTypeDef status =
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_addr, reinterpret_cast<uint32_t>(data));
  HAL_FLASH_Lock();

  return status == HAL_OK;
}

uint32_t row_addr_to_block(uint32_t row_addr) {
  return row_addr / NAND_PAGES_PER_BLOCK;
}

uint32_t first_row_addr_in_block(uint32_t row_addr) {
  return row_addr_to_block(row_addr) * NAND_PAGES_PER_BLOCK;
}

// Each entry takes a whole flash word, since a flash word can only be programmed once
uint8_t *get_entry_addr(uint32_t index) {
  return bpt_entries + index * STM32_FLASH_WORD_SIZE;
}

uint32_t get_entry(uint32_t index) {
  return *reinterpret_cast<uint32_t *>(get_entry_addr(index));
}

uint32_t get_bpt_bad_byte_index(uint32_t block_num) {
  return block_num / 8;
}

uint8_t get_bpt_bad_bit_mask(uint32_t block_num) {
  return 1u << (block_num % 8);
}

// Whether the block was marked bad by the factory
bool is_factory_bad_block(uint32_t block_num) {
  return !(bpt_factory_table[get_bpt_bad_byte_index(block_num)] & get_bpt_bad_bit_mask(block_num));
}

// Checks if a page in the block was marked bad at runtime. If so, returns true and
// sets out_row_addr to the first page in the block that went bad. Pages before it
// still hold valid data, and it and every page after it are unusable
bool get_runtime_bad_page(uint32_t block_num, uint32_t *out_row_addr) {
  bool found = false;

  for (uint32_t i = 0; i < BPT_MAX_ENTRIES; i++) {
    uint32_t entry = get_entry(i);

    // entries are filled in order, so no more entries after an empty one
    if (entry == BPT_EMPTY_ENTRY) {
      break;
    }

    if (row_addr_to_block(entry) != block_num) {
      continue;
    }

    if (!found || entry < *out_row_addr) {
      *out_row_addr = entry;
      found = true;
    }
  }

  return found;
}

// Stores the row address of the specific page that went bad in the next empty entry
bool add_entry(uint32_t row_addr) {
  for (uint32_t i = 0; i < BPT_MAX_ENTRIES; i++) {
    if (get_entry(i) != BPT_EMPTY_ENTRY) {
      continue;
    }

    // entry is the row address, padded out to a whole flash word
    uint32_t word_buf[STM32_FLASH_WORD_U32S];
    memset(word_buf, ERASED_BYTE, sizeof(word_buf));
    word_buf[0] = row_addr;

    if (!program_flash_word(reinterpret_cast<uint32_t>(get_entry_addr(i)), word_buf)) {
      CommsSerial.printf("WARNING: failed to mark page %lu (block %lu) bad.\n", row_addr, row_addr_to_block(row_addr));
      return false;
    }

    return true;
  }

  CommsSerial.printf("WARNING: Bad Page Table full, block %lu not marked bad.\n", row_addr_to_block(row_addr));
  return false;
}

// Reads every block's factory bad block mark and writes the factory table.
// Must run before the NAND is ever erased, since erasing wipes the marks
bool scan_factory_bad_blocks() {
  static uint32_t table_buf[BPT_FACTORY_TABLE_SIZE / sizeof(uint32_t)];
  uint8_t *table_bytes = reinterpret_cast<uint8_t *>(table_buf);
  memset(table_buf, ERASED_BYTE, sizeof(table_buf));

  for (uint32_t block_num = 0; block_num < NAND_NUM_BLOCKS; block_num++) {
    uint32_t row_addr = block_num * NAND_PAGES_PER_BLOCK;

    // mark is the first byte of the spare area of the block's first page
    uint8_t bad_block_mark;
    flash_error_t err = Flash::read(row_addr, NAND_MAIN_PAGE_SIZE, &bad_block_mark, 1);

    if (err != FLASH_SUCCESS) {
      return false; 
    }

    // a factory bad block might not even read back correctly
    if (bad_block_mark != NAND_GOOD_BLOCK_MARK) {
      table_bytes[get_bpt_bad_byte_index(block_num)] &= ~get_bpt_bad_bit_mask(block_num);
    }
  }

  // flash can only be programmed once per erase, in 32 byte aligned chunks
  for (uint32_t offset = 0; offset < BPT_FACTORY_TABLE_SIZE; offset += STM32_FLASH_WORD_SIZE) {
    uint32_t flash_addr = reinterpret_cast<uint32_t>(bpt_factory_table + offset);
    if (!program_flash_word(flash_addr, table_buf + offset / sizeof(uint32_t))) {
      return false;
    }
  }

  return true;
}

bool user_confirm_overwrite() {
  const int key_min = 1000;
  const int key_max = 10000;
  srand(millis());
  int key = rand() % (key_max - key_min) + key_min;

  CommsSerial.println("WARNING: Bad Page Table not initialized - a factory bad-block scan is required.");
  CommsSerial.printf("Enter %d to proceed: \n", key);

  char *res = CommsSerial.readline();
  if (atoi(res) != key) {
    CommsSerial.println("Confirmation failed");
    return false;
  }

  return true;
}

bool erase_table() {
  FLASH_EraseInitTypeDef erase_init = {
    .TypeErase = FLASH_TYPEERASE_SECTORS,
    .Banks = BPT_BANK,
    .Sector = BPT_SECTOR,
    .NbSectors = BPT_NUM_SECTORS,
    .VoltageRange = BPT_VOLTAGE_RANGE
  };
  uint32_t sector_error;

  HAL_FLASH_Unlock();
  bool erase_ok = HAL_FLASHEx_Erase(&erase_init, &sector_error) == HAL_OK;
  HAL_FLASH_Lock();

  return erase_ok;
}

} // namespace

bool begin() {
  // magic number is only written once the factory scan finished,
  // so the table is already set up
  if (*bpt_magic == BPT_MAGIC_NUMBER) {
    return true;
  }

  if (!user_confirm_overwrite()) {
    return false;
  }

  if (!erase_table()) {
    CommsSerial.println("Bad Page Table erase failed.");
    return false;
  }

  CommsSerial.println("Starting scan for factory bad blocks. This may take a while.");

  if (!scan_factory_bad_blocks()) {
    CommsSerial.println("Factory bad-block scan failed.");
    return false;
  }

  // written last, so a scan interrupted by a reset is redone on the next boot
  uint32_t word_buf[STM32_FLASH_WORD_U32S];
  memset(word_buf, ERASED_BYTE, sizeof(word_buf));
  word_buf[0] = BPT_MAGIC_NUMBER;

  if (!program_flash_word(reinterpret_cast<uint32_t>(bpt_magic), word_buf)) {
    CommsSerial.println("Flash program failed");
    return false;
  }

  CommsSerial.println("Bad Page Table initialized.");
  return true;
}

page_status_t get_page_status(uint32_t row_addr) {
  if (row_addr >= NAND_NUM_PAGES) {
    return PAGE_BAD;
  }

  uint32_t block_num = row_addr_to_block(row_addr);
  if (is_factory_bad_block(block_num)) {
    return PAGE_BAD;
  }

  uint32_t first_bad_row_addr;
  if (!get_runtime_bad_page(block_num, &first_bad_row_addr)) {
    return PAGE_GOOD;
  }

  if (row_addr < first_bad_row_addr) {
    return PAGE_READ_ONLY;
  }

  return PAGE_BAD;
}

bool mark_page_bad(uint32_t row_addr) {
  if (row_addr >= NAND_NUM_PAGES) {
    return false;
  }

  // already marked
  if (get_page_status(row_addr) != PAGE_GOOD) {
    return true;
  }

  return add_entry(row_addr);
}

bool retire_block(uint32_t row_addr) {
  if (row_addr >= NAND_NUM_PAGES) {
    return false;
  }

  // marking the first page bad makes every page in the block bad
  uint32_t first_row_addr = first_row_addr_in_block(row_addr);
  if (get_page_status(first_row_addr) == PAGE_BAD) {
    return true;
  }

  return add_entry(first_row_addr);
}

} // namespace Bad_Page_Table
