/**
 * @file logging.cpp
 * @brief Packed struct logging to NAND flash for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include <Arduino.h>

#include "logging.h"
#include "CommsSerial.h"

#include <string.h>

#define DUMP_CONTINUE_CHAR 'c'
#define DUMP_TIMEOUT_MS 5000u

namespace Logging {

namespace {

uint8_t page_buf[NAND_PAGE_SIZE];
uint8_t read_buf[NAND_PAGE_SIZE];
uint32_t page_offset = 0;
uint32_t cur_page = NAND_NUM_PAGES; // init to one past the last valid page

uint32_t skip_to_next_block(uint32_t page) {
  return (page / NAND_PAGES_PER_BLOCK + 1) * NAND_PAGES_PER_BLOCK;
}

uint32_t first_good_page_from(uint32_t page) {
  while (page < NAND_NUM_PAGES && !Bad_Page_Table::is_page_good(page)) {
    // if one page in a block is bad, then all 64 pages are marked bad
    // and we can just skip to the next block
    page = skip_to_next_block(page);
  }

  return page;
}

void reset_page_buf() {
  memset(page_buf, LOG_EMPTY_BYTE, sizeof(page_buf));
  page_offset = 0;
}

bool page_is_empty(uint32_t page) {
  uint8_t first_byte;
  if (Flash::read_main(page, 0, &first_byte, 1) != FLASH_SUCCESS) {
    return false;
  }

  return first_byte == LOG_EMPTY_BYTE;
}

// Find the first empty good page to resume writing
uint32_t find_write_page() {
  uint32_t last_used_block = UINT32_MAX;

  for (uint32_t block = 0; block < NAND_NUM_BLOCKS; block++) {
    uint32_t first_page = block * NAND_PAGES_PER_BLOCK;
    if (!Bad_Page_Table::is_page_good(first_page)) {
      continue;
    }

    if (page_is_empty(first_page)) {
      break;
    }

    last_used_block = block;
  }

  if (last_used_block == UINT32_MAX) {
    return first_good_page_from(0);
  }

  uint32_t page = last_used_block * NAND_PAGES_PER_BLOCK;
  uint32_t block_end = page + NAND_PAGES_PER_BLOCK;
  while (page < block_end && !page_is_empty(page)) {
    page++;
  }

  return first_good_page_from(page);
}

bool commit_page() {
  if (Flash::write_to_cache(0, page_buf, NAND_PAGE_SIZE) != FLASH_SUCCESS) {
    CommsSerial.println("Write to cache failed");
    return false;
  }
  
  flash_error_t err = Flash::program(cur_page);
  
  if (err == FLASH_BAD_BLOCK) {
    if (!Bad_Page_Table::mark_page_bad(cur_page)) {
      CommsSerial.printf("While committing page %lu, it was found to be bad but marking it as such failed\n", cur_page);
      
      // skip to the next block manually
      cur_page = skip_to_next_block(cur_page);
    }
    cur_page = first_good_page_from(cur_page);
  }

  if (cur_page >= NAND_NUM_PAGES) {
    return false; 
  }

  if (err != FLASH_SUCCESS) {
    return false;
  }

  reset_page_buf();
  cur_page = first_good_page_from(cur_page + 1);

  return true;
}

bool wait_for_continue() {
  uint32_t start = millis();

  while (!CommsSerial.available()) {
    if (millis() - start > DUMP_TIMEOUT_MS) {
      return false;
    }
  }

  return CommsSerial.read() == DUMP_CONTINUE_CHAR;
}

flash_error_t read_page(uint32_t page_addr) {
  if (!Bad_Page_Table::is_page_good(page_addr)) {
    return FLASH_FAIL;
  }

  flash_error_t err = Flash::read_main(page_addr, 0, read_buf, NAND_PAGE_SIZE);
  
  if (err == FLASH_BAD_BLOCK) {
    Bad_Page_Table::mark_page_bad(page_addr);
  }

  return err;
}

bool send_page() {
  if (!wait_for_continue()) {
    return false;
  }

  uint8_t checksum_a = 0;
  uint8_t checksum_b = 0;
  for (uint32_t i = 0; i < NAND_PAGE_SIZE; i++) {
    checksum_a += read_buf[i];
    checksum_b += checksum_a;
  }

  CommsSerial.write(read_buf, NAND_PAGE_SIZE);
  CommsSerial.write(checksum_a);
  CommsSerial.write(checksum_b);
  return true;
}

} // namespace

bool begin() {
  if (!Flash::begin()) {
    CommsSerial.println("Flash init failed");
    return false;
  }

  if (!Bad_Page_Table::begin()) {
    CommsSerial.println("Bad Page Table Init failed");
    return false;
  }

  reset_page_buf();
  
  cur_page = find_write_page();

  if (cur_page >= NAND_NUM_PAGES) {
    CommsSerial.println("NAND log is full.");
    return false;
  }

  return true;
}

bool erase() {
  bool ok = true;

  CommsSerial.println("Erasing NAND log. This may take a while.");

  // loop through all blocks and erase filled blocks
  for (uint32_t block = 0; block < NAND_NUM_BLOCKS; block++) {
    uint32_t first_page = block * NAND_PAGES_PER_BLOCK;
    if (!Bad_Page_Table::is_page_good(first_page)) {
      continue;
    }

    flash_error_t err = Flash::erase_block(first_page);
    if (err == FLASH_BAD_BLOCK) {
      // only takes one bad page marking to fail
      ok &= Bad_Page_Table::mark_page_bad(first_page);
    }

    if (err != FLASH_SUCCESS) {
      ok = false;
    }
  }

  reset_page_buf();
  cur_page = first_good_page_from(0);

  if (cur_page >= NAND_NUM_PAGES) {
    CommsSerial.println("Something is very wrong, first good page is outside flash");
    return false;
  }

  CommsSerial.println("NAND log erased.");
  return ok;
}

bool write(uint8_t struct_id, uint8_t *data, size_t len) {
  size_t entry_len = sizeof(struct_id) + len;
  if (struct_id == LOG_EMPTY_BYTE || entry_len > NAND_PAGE_SIZE) {
    return false;
  }

  if (page_offset + entry_len > NAND_PAGE_SIZE) {
    if (!commit_page()) {
      return false;
    }
  }

  if (cur_page >= NAND_NUM_PAGES) {
    return false;
  }

  page_buf[page_offset] = struct_id;
  memcpy(page_buf + page_offset + sizeof(struct_id), data, len);
  page_offset += entry_len;

  return true;
}

bool flush() {
  // Either buffer is empty and nothing to flush or
  // flash is full and cannot commit another page
  if (page_offset == 0 || cur_page >= NAND_NUM_PAGES) {
    return true;
  }

  return commit_page();
}

bool dump() {
  if (!flush()) {
    return false;
  }

  for (uint32_t page = first_good_page_from(0); page < NAND_NUM_PAGES; page = first_good_page_from(page + 1)) {
    if (read_page(page) != FLASH_SUCCESS) {
      continue;
    }

    if (read_buf[0] == LOG_EMPTY_BYTE) {
      break;
    }

    if (!send_page()) {
      return false;
    }
  }

  memset(read_buf, LOG_EMPTY_BYTE, sizeof(read_buf));
  return send_page();
}

} // namespace Logging
