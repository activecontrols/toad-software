/**
 * @file logging.cpp
 * @brief Packed struct logging to NAND flash for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include <Arduino.h>

#include "logging.h"
#include "CommandRouter.h"
#include "CommsSerial.h"

#include <string.h>

#define DUMP_CONTINUE_CHAR  'c'
#define DUMP_KILL_CHAR      'k'
#define DUMP_TIMEOUT_MS     5000u
#define LOG_FIRST_ROW_ADDR  0u
#define PAGE_FIRST_COL_ADDR 0u
#define DumpSerial          SerialUSB

#define RETURN_IF_FAILED(item, failure_case, failure_return)          \
  do {                                                                \
    if ((item) == (failure_case)) {                                   \
      return (failure_return);                                        \
    }                                                                 \
  } while(0)

namespace Logging {

namespace {

// Two page buffers, so one can be filled with new log entries
// while the other is still being programmed into flash
uint8_t page_bufs[2][NAND_MAIN_PAGE_SIZE];
uint8_t *page_buf = page_bufs[0];
uint32_t page_offset = 0;

// Next row address (page) to be programmed
// init to one past the last valid page
uint32_t cur_row_addr = NAND_NUM_PAGES;

// The page that was last programmed, but not yet checked for success.
// pending_buf is kept around so the page can be retried if the program fails
uint8_t *pending_buf = nullptr;
uint32_t pending_row_addr = NAND_NUM_PAGES;

// Holds a page read back from flash during a dump
uint8_t read_buf[NAND_MAIN_PAGE_SIZE];

bool in_flight = false;

uint32_t skip_to_next_block(uint32_t row_addr) {
  return (row_addr / NAND_PAGES_PER_BLOCK + 1) * NAND_PAGES_PER_BLOCK;
}

uint32_t first_good_page_from(uint32_t row_addr) {
  while (row_addr < NAND_NUM_PAGES && Bad_Page_Table::get_page_status(row_addr) != PAGE_GOOD) {
    // if one page in a block is bad, then all 64 pages are marked bad
    // and we can just skip to the next block
    row_addr = skip_to_next_block(row_addr);
  }

  return row_addr;
}

bool flash_is_full() {
  return cur_row_addr >= NAND_NUM_PAGES;
}

bool program_pending() {
  return pending_buf != nullptr;
}

void reset_page_buf() {
  memset(page_buf, LOG_EMPTY_BYTE, NAND_MAIN_PAGE_SIZE);
  page_offset = 0;
}

void swap_page_buf() {
  if (page_buf == page_bufs[0]) {
    page_buf = page_bufs[1];
  } else {
    page_buf = page_bufs[0];
  }

  reset_page_buf();
}

bool page_is_empty(uint32_t row_addr) {
  uint8_t first_byte;

  // if the page can't be read, assume it has data so it is never overwritten
  if (Flash::read(row_addr, PAGE_FIRST_COL_ADDR, &first_byte, 1) != FLASH_SUCCESS) {
    return false;
  }

  // every written page starts with a struct id, which is never 0xFF
  return first_byte == LOG_EMPTY_BYTE;
}

bool find_empty_page_in_block(uint32_t block_row_addr, uint32_t *out_row_addr) {
  RETURN_IF_FAILED(out_row_addr, nullptr, false);

  uint32_t next_block = skip_to_next_block(block_row_addr);

  for (uint32_t row_addr = block_row_addr; row_addr < next_block; row_addr++) {
    if (page_is_empty(row_addr)) {
      *out_row_addr = row_addr;
      return true;
    }
  }

  // block is full
  return false;
}

uint32_t first_empty_good_page() {
  uint32_t last_used_block = NAND_NUM_PAGES;
  uint32_t block_row_addr = first_good_page_from(LOG_FIRST_ROW_ADDR);

  // pages are written in order, so find the first good block
  // that has nothing written to its first page
  while (block_row_addr < NAND_NUM_PAGES && !page_is_empty(block_row_addr)) {
    last_used_block = block_row_addr;
    block_row_addr = first_good_page_from(skip_to_next_block(block_row_addr));
  }

  // flash is completely empty
  if (last_used_block == NAND_NUM_PAGES) {
    return block_row_addr;
  }

  // go back to the last used block & check if an empty page exists within it
  uint32_t row_addr;
  if (find_empty_page_in_block(last_used_block, &row_addr)) {
    return row_addr;
  }

  return block_row_addr;
}

bool start_program(uint8_t *buf) {
  // load the whole page into the NAND's cache
  if (Flash::write_to_cache(PAGE_FIRST_COL_ADDR, buf, NAND_MAIN_PAGE_SIZE) != FLASH_SUCCESS) {
    CommsSerial.println("Write to cache failed");
    return false;
  }

  // start writing the cache into the page, but don't wait for it to finish
  if (Flash::program(cur_row_addr) != FLASH_SUCCESS) {
    CommsSerial.println("Program execute failed");
    return false;
  }

  // remember this page, so it can be checked later
  pending_buf = buf;
  pending_row_addr = cur_row_addr;

  cur_row_addr = first_good_page_from(cur_row_addr + 1);

  return true;
}

flash_error_t wait_for_program() {
  uint32_t start_time = micros();

  while (true) {
    flash_error_t err = Flash::check_program_status();

    // program finished, either successfully or not
    if (err != FLASH_BUSY) {
      return err;
    }

    if (micros() - start_time > WORST_CASE_PROGRAM_TIMEOUT) {
      return FLASH_TIMED_OUT;
    }
  }
}

bool handle_bad_program() {
  // only program/erase failures mean the block is worn out
  if (!Bad_Page_Table::mark_page_bad(pending_row_addr)) {
    CommsSerial.printf("While committing page %lu, it was found to be bad but marking it as such failed\n", pending_row_addr);
  }

  // pages in a block must be programmed in order,
  // so move on to the next good block
  cur_row_addr = first_good_page_from(skip_to_next_block(pending_row_addr));

  // pending buffer should always point at a buffer with an unknown status
  uint8_t *retry_buf = pending_buf;
  pending_buf = nullptr;

  RETURN_IF_FAILED(flash_is_full(), true, false);

  // the failed page's data is still in its buffer, so program it again
  return start_program(retry_buf);
}

bool resolve_pending_program() {
  // keep retrying until the pending page is written to a good block
  while (program_pending()) {
    flash_error_t err = wait_for_program();

    if (err == FLASH_SUCCESS) {
      pending_buf = nullptr;
      return true;
    }

    // timed out or failed to talk to the chip,
    // keep the page pending so it can be checked again later
    if (err != FLASH_BAD_BLOCK) {
      return false;
    }

    RETURN_IF_FAILED(handle_bad_program(), false, false);
  }

  return true;
}

bool commit_page() {
  // previous page must be done before the cache can be loaded again
  RETURN_IF_FAILED(resolve_pending_program(), false, false);
  RETURN_IF_FAILED(flash_is_full(), true, false);
  RETURN_IF_FAILED(start_program(page_buf), false, false);

  // start filling the other buffer while this one is programmed
  swap_page_buf();

  return true;
}

bool wait_for_continue() {
  uint32_t start = millis();

  while (!DumpSerial.available()) {
    if (millis() - start > DUMP_TIMEOUT_MS) {
      return false;
    }
  }

  return DumpSerial.read() == DUMP_CONTINUE_CHAR;
}

flash_error_t read_page(uint32_t row_addr) {
  return Flash::read(row_addr, PAGE_FIRST_COL_ADDR, read_buf, NAND_MAIN_PAGE_SIZE);
}

bool send_page_to_host(uint32_t row_addr) {
  RETURN_IF_FAILED(wait_for_continue(), false, false);

  // Fletcher-16 checksum so the host can verify the page arrived intact
  uint8_t checksum_a = 0;
  uint8_t checksum_b = 0;
  for (uint32_t i = 0; i < NAND_MAIN_PAGE_SIZE; i++) {
    checksum_a += read_buf[i];
    checksum_b += checksum_a;
  }

  DumpSerial.write(DUMP_CONTINUE_CHAR);
  DumpSerial.write(reinterpret_cast<uint8_t *>(&row_addr), sizeof(row_addr));
  DumpSerial.write(read_buf, NAND_MAIN_PAGE_SIZE);
  DumpSerial.write(checksum_a);
  DumpSerial.write(checksum_b);
  return true;
}

void send_end_page() {
  if (wait_for_continue()) {
    DumpSerial.write(DUMP_KILL_CHAR);
  }
}

bool send_log() {
  uint32_t row_addr = LOG_FIRST_ROW_ADDR;

  while (row_addr < NAND_NUM_PAGES) {
    page_status_t status = Bad_Page_Table::get_page_status(row_addr);

    // nothing to read in a bad block
    if (status == PAGE_BAD) {
      row_addr = skip_to_next_block(row_addr);
      continue;
    }

    flash_error_t err = read_page(row_addr);

    // data on this page is corrupted, skip it
    if (err == FLASH_ECC_FAIL) {
      row_addr++;
      continue;
    }

    if (err != FLASH_SUCCESS) {
      return false;
    }

    if (read_buf[0] == LOG_EMPTY_BYTE) {
      // end of logs found
      if (status == PAGE_GOOD) {
        break;
      }

      // otherwise, page is read only (the result of 
      // this block being marked bad by some bad page), 
      // go to next block
      row_addr = skip_to_next_block(row_addr);
      continue;
    }

    RETURN_IF_FAILED(send_page_to_host(row_addr), false, false);

    row_addr++;
  }

  send_end_page();
  return true;
}

void dump_log_cmd() {
  // never dump during flight, it blocks for a long time
  if (in_flight) {
    return;
  }

  if (!dump()) {
    CommsSerial.println("Log dump failed");
    return;
  }

  CommsSerial.println("Log dump finished");
}

void erase_log_cmd() {
  // never erase during flight
  if (in_flight) {
    return;
  }

  erase();
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

  // append to the existing log instead of overwriting it
  cur_row_addr = first_empty_good_page();

  if (flash_is_full()) {
    CommsSerial.println("WARNING: NAND log is full, erase it before logging.");
  }

  CommandRouter::add(dump_log_cmd, "dump_logs", "Dump the NAND log over USB serial.");
  CommandRouter::add(erase_log_cmd, "erase_logs", "Erase the NAND log.");

  return true;
}

void set_in_flight(bool flying) {
  in_flight = flying;
}

bool erase() {
  bool ok = true;

  CommsSerial.println("Erasing NAND log. This may take a while.");

  // whatever was pending is about to be erased anyway
  pending_buf = nullptr;

  // loop through all blocks and erase filled blocks
  for (uint32_t block = 0; block < NAND_NUM_BLOCKS; block++) {
    uint32_t first_row_addr = block * NAND_PAGES_PER_BLOCK;
    page_status_t status = Bad_Page_Table::get_page_status(first_row_addr);

    if (status == PAGE_BAD) {
      continue;
    }

    // block had a page fail while logging, erase it and never use it again
    if (status == PAGE_READ_ONLY) {
      Flash::erase_block(first_row_addr);
      ok &= Bad_Page_Table::retire_block(first_row_addr);
      continue;
    }

    flash_error_t err = Flash::erase_block(first_row_addr);

    if (err == FLASH_BAD_BLOCK) {
      // only takes one bad page marking to fail
      ok &= Bad_Page_Table::mark_page_bad(first_row_addr);
    }

    if (err != FLASH_SUCCESS) {
      ok = false;
    }
  }

  reset_page_buf();
  cur_row_addr = first_good_page_from(LOG_FIRST_ROW_ADDR);

  if (flash_is_full()) {
    CommsSerial.println("Something is very wrong, first good page is outside flash");
    return false;
  }

  CommsSerial.println("NAND log erased.");
  return ok;
}

bool write(uint8_t struct_id, uint8_t *data, size_t len) {
  RETURN_IF_FAILED(data, nullptr, false);

  size_t entry_len = sizeof(struct_id) + len;

  // 0xFF marks the end of a page, so it can't be used as an id
  if (struct_id == LOG_EMPTY_BYTE || entry_len > NAND_MAIN_PAGE_SIZE || len == 0) {
    return false;
  }

  // entries never cross a page boundary,
  // so commit the page if this entry won't fit
  if (page_offset + entry_len > NAND_MAIN_PAGE_SIZE) {
    RETURN_IF_FAILED(commit_page(), false, false);
  }

  RETURN_IF_FAILED(flash_is_full(), true, false);

  // entry is the id byte followed by the packed struct
  page_buf[page_offset] = struct_id;
  memcpy(page_buf + page_offset + sizeof(struct_id), data, len);
  page_offset += entry_len;

  return true;
}

bool flush() {
  // commit whatever is in the buffer, if there is anything and room for it
  if (page_offset != 0 && !flash_is_full()) {
    RETURN_IF_FAILED(commit_page(), false, false);
  }

  // make sure the last page actually made it into flash
  return resolve_pending_program();
}

bool dump() {
  RETURN_IF_FAILED(flush(), false, false);

  return send_log();
}

} // namespace Logging
