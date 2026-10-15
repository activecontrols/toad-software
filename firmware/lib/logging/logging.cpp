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
uint32_t cur_page = NAND_NUM_PAGES;

uint32_t next_good_page(uint32_t page) {
  while (page < NAND_NUM_PAGES && !Bad_Page_Table::is_page_good(page)) {
    page = (page / NAND_PAGES_PER_BLOCK + 1) * NAND_PAGES_PER_BLOCK;
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
    return next_good_page(0);
  }

  uint32_t page = last_used_block * NAND_PAGES_PER_BLOCK;
  uint32_t block_end = page + NAND_PAGES_PER_BLOCK;
  while (page < block_end && !page_is_empty(page)) {
    page++;
  }

  return next_good_page(page);
}

bool commit_page() {
  bool ok = true;

  if (Flash::write_to_cache(0, page_buf, NAND_PAGE_SIZE) != FLASH_SUCCESS) {
    ok = false;
  } else if (Flash::program(cur_page) != FLASH_SUCCESS) {
    Bad_Page_Table::mark_page_bad(cur_page);
    ok = false;
  }

  reset_page_buf();
  cur_page = next_good_page(cur_page + 1);
  return ok;
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

bool send_page(uint8_t *data) {
  if (!wait_for_continue()) {
    return false;
  }

  uint8_t cs_a = 0;
  uint8_t cs_b = 0;
  for (uint32_t i = 0; i < NAND_PAGE_SIZE; i++) {
    cs_a += data[i];
    cs_b += cs_a;
  }

  CommsSerial.write(data, NAND_PAGE_SIZE);
  CommsSerial.write(cs_a);
  CommsSerial.write(cs_b);
  return true;
}

} // namespace

bool begin() {
  if (!Bad_Page_Table::begin()) {
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

  for (uint32_t block = 0; block < NAND_NUM_BLOCKS; block++) {
    uint32_t first_page = block * NAND_PAGES_PER_BLOCK;
    if (!Bad_Page_Table::is_page_good(first_page)) {
      continue;
    }

    if (Flash::erase_block(first_page) != FLASH_SUCCESS) {
      ok &= Bad_Page_Table::mark_page_bad(first_page);
    }
  }

  reset_page_buf();
  cur_page = next_good_page(0);

  CommsSerial.println("NAND log erased.");
  return ok;
}

bool write(uint8_t struct_id, uint8_t *data, size_t len) {
  size_t entry_len = sizeof(struct_id) + len;
  if (struct_id == LOG_EMPTY_BYTE || entry_len > NAND_PAGE_SIZE) {
    return false;
  }

  bool ok = true;
  if (page_offset + entry_len > NAND_PAGE_SIZE) {
    ok = commit_page();
  }

  if (cur_page >= NAND_NUM_PAGES) {
    return false;
  }

  page_buf[page_offset] = struct_id;
  memcpy(page_buf + page_offset + sizeof(struct_id), data, len);
  page_offset += entry_len;

  return ok;
}

bool flush() {
  if (page_offset == 0 || cur_page >= NAND_NUM_PAGES) {
    return true;
  }

  return commit_page();
}

flash_error_t read_page(uint32_t page_addr, uint8_t *out) {
  if (!Bad_Page_Table::is_page_good(page_addr)) {
    return FLASH_FAIL;
  }

  flash_error_t err = Flash::read_main(page_addr, 0, out, NAND_PAGE_SIZE);
  if (err == FLASH_ECC_FAIL) {
    Bad_Page_Table::mark_page_bad(page_addr);
  }

  return err;
}

bool dump() {
  flush();

  for (uint32_t page = next_good_page(0); page < NAND_NUM_PAGES; page = next_good_page(page + 1)) {
    if (read_page(page, read_buf) != FLASH_SUCCESS) {
      continue;
    }

    if (read_buf[0] == LOG_EMPTY_BYTE) {
      break;
    }

    if (!send_page(read_buf)) {
      return false;
    }
  }

  memset(read_buf, LOG_EMPTY_BYTE, sizeof(read_buf));
  return send_page(read_buf);
}

} // namespace Logging
