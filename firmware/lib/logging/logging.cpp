#include "logging.h"
#include "bad_page_table.h"
#include "flash_defs.h"

#include "CommandRouter.h"
#include "CommsSerial.h"

#include <Arduino.h>
#include <stdlib.h>

namespace Logging {

bool log_enabled = false;
uint32_t write_page_addr = 0; // page index currently being assembled in the NAND's cache
uint32_t write_col_addr = 0;  // byte offset within that page's cache

bool is_armed() {
  return log_enabled;
}

// Advances write_page_addr to the next page whose containing block is not marked bad.
// Returns false if there are no usable pages left in the NAND.
bool advance_to_next_good_page() {
  write_page_addr++;

  while (write_page_addr < NAND_NUM_PAGES &&
         !BPT::is_page_good(write_page_addr)) {
    write_page_addr++;
  }

  write_col_addr = 0;

  return write_page_addr < NAND_NUM_PAGES;
}

// Programs the current NAND cache into the page being assembled. If programming fails,
// the containing block is marked bad so it will not be used again.
static flash_error_t flush_page() {
  if (write_col_addr == 0)
    return FLASH_ERROR_SUCCESS;

  flash_error_t err =
      Flash::program(write_page_addr);

  if (err != FLASH_ERROR_SUCCESS) {
    BPT::mark_page_bad(write_page_addr);
  }

  return err;
}

void write(uint8_t struct_id, uint8_t *data, uint32_t len) {
  if (!log_enabled)
    return;

  uint32_t entry_len = 1u + len; // id byte + struct data

  if (entry_len > FLASH_PAGE_SIZE) {
    CommsSerial.println(
        "Logging::write - struct too large to fit in one page, dropped");
    return;
  }

  // Never split an entry across a page boundary. If the current page does not have
  // enough remaining space, program it and continue on the next usable page.
  if (write_col_addr + entry_len > FLASH_PAGE_SIZE) {
    if (flush_page() != FLASH_ERROR_SUCCESS) {
      CommsSerial.println(
          "Logging::write - page program failed, disarming");
      log_enabled = false;
      return;
    }

    if (!advance_to_next_good_page()) {
      CommsSerial.println(
          "Logging::write - out of flash, disarming");
      log_enabled = false;
      return;
    }
  }

  flash_error_t err =
      Flash::write_to_cache(write_col_addr, &struct_id, 1);

  if (err != FLASH_ERROR_SUCCESS) {
    CommsSerial.println(
        "Logging::write - failed to write struct id to cache");
    log_enabled = false;
    return;
  }

  write_col_addr++;

  err =
      Flash::write_to_cache(write_col_addr, data, len);

  if (err != FLASH_ERROR_SUCCESS) {
    CommsSerial.println(
        "Logging::write - failed to write struct data to cache");
    log_enabled = false;
    return;
  }

  write_col_addr += len;
}

void complete() {
  if (!log_enabled)
    return;

  if (flush_page() != FLASH_ERROR_SUCCESS) {
    CommsSerial.println(
        "Logging::complete - failed to program final page");
  }

  log_enabled = false;
}

flash_error_t read_page(uint32_t addr, uint32_t *data) {
  if (!BPT::is_page_good(addr))
    return FLASH_ERROR_FAIL;

  flash_error_t err =
      Flash::read_page(
          addr,
          reinterpret_cast<uint8_t *>(data));

  // TODO: Only mark the containing block bad when Flash::read_page can distinguish
  // an uncorrectable ECC/media failure from a timeout, QSPI error, or other transient
  // communication failure.
  //
  // Daniel's current Flash::read_page implementation also needs to distinguish
  // corrected ECC errors from uncorrectable ECC errors before this can be handled
  // correctly here.

  return err;
}

// Requires the factory bad-block scan to be complete before allowing any erase/program
// operations. A random confirmation key is also required because arming the logger erases
// all currently-good NAND blocks.
void cmd_log_arm() {
  if (log_enabled) {
    CommsSerial.println("Log is already enabled.");
    return;
  }

  if (!BPT::factory_scan_complete()) {
    CommsSerial.println(
        "Cannot arm logging before factory bad-block scan.");
    CommsSerial.println(
        "Run factory_scan first.");
    return;
  }

  const int key_min = 1000;
  const int key_max = 10000;

  srand(millis());

  int key =
      rand() % (key_max - key_min) + key_min;

  CommsSerial.println(
      "WARNING: Proceeding will erase all data currently on the NAND.");

  CommsSerial.printf(
      "Enter %d to proceed: \n",
      key);

  char *res =
      CommsSerial.readline();

  if (atoi(res) != key) {
    CommsSerial.println(
        "Incorrect key entered.");
    return;
  }

  CommsSerial.println(
      "Erasing all good blocks. This may take a while.");

  for (uint32_t block = 0;
       block < NAND_NUM_BLOCKS;
       ++block) {

    uint32_t page_addr =
        block * NAND_PAGES_PER_BLOCK;

    if (!BPT::is_page_good(page_addr))
      continue;

    if (Flash::erase_block(page_addr) != FLASH_ERROR_SUCCESS) {
      BPT::mark_page_bad(page_addr);
    }
  }

  write_page_addr = 0;

  while (write_page_addr < NAND_NUM_PAGES &&
         !BPT::is_page_good(write_page_addr)) {
    write_page_addr++;
  }

  if (write_page_addr >= NAND_NUM_PAGES) {
    CommsSerial.println(
        "No usable NAND blocks available.");
    return;
  }

  write_col_addr = 0;
  log_enabled = true;

  CommsSerial.println(
      "Erase complete. Logging enabled.");
}

// Streams the log out page-by-page over USB serial, mirroring ASTRA's
// send_flash_over_serial() protocol. Each page is followed by a two-byte additive
// checksum and a 'c'/'k' handshake. An all-0xFF page is treated as the end of the log.
//
// Walking page indices in order and skipping blocks currently marked bad reproduces the
// same page order used while logging, as long as the BPT has not changed since the log
// was written.
void dump_flash() {
  static uint8_t page_buf[FLASH_PAGE_SIZE];

  while (USB_CommsSerial.available()) {
    USB_CommsSerial.read();
  }

  for (uint32_t addr = 0;
       addr < NAND_NUM_PAGES;
       ++addr) {

    if (!BPT::is_page_good(addr))
      continue;

    flash_error_t err =
        read_page(
            addr,
            reinterpret_cast<uint32_t *>(page_buf));

    if (err != FLASH_ERROR_SUCCESS) {
      continue;
    }

    bool all_ff = true;
    uint8_t cs_a = 0;
    uint8_t cs_b = 0;

    for (uint32_t i = 0;
         i < FLASH_PAGE_SIZE;
         ++i) {

      cs_a += page_buf[i];
      cs_b += cs_a;

      if (page_buf[i] != 0xFF)
        all_ff = false;
    }

    if (all_ff) {
      USB_CommsSerial.write('k');
      return;
    }

    USB_CommsSerial.write(
        page_buf,
        sizeof(page_buf));

    USB_CommsSerial.write(cs_a);
    USB_CommsSerial.write(cs_b);
    USB_CommsSerial.write('c');

    int c;
    uint32_t wait_start =
        millis();

    do {
      c =
          USB_CommsSerial.read();

      if (millis() - wait_start > 5000) {
        CommsSerial.println(
            "Error: flash dump timed out");
        return;
      }

    } while (!(c == 'k' || c == 'c'));

    if (c == 'k')
      return;
  }

  USB_CommsSerial.write('k');
}

// TODO: Factory bad-block scanning requires access to the NAND spare/OOB area.
// The factory bad-block marker is stored at column 2048 of the first page of each
// block, which is outside the 2048-byte main-data region exposed by Flash::read_page().
//
// Daniel's flash driver needs to expose a low-level cache/spare-area read, such as
// read_from_cache(column, data, len) or read_oob_byte(page, column), before this can
// be implemented.
//
// Once that primitive exists, this function should:
//   - inspect the first page of every NAND block
//   - read the factory marker at spare-area byte 2048
//   - treat any value other than 0xFF as a bad block
//   - store the resulting bad-block information in the BPT
//   - call BPT::mark_factory_scan_complete() only after the entire scan succeeds
//
// No NAND erase or program operation should be allowed before this scan completes.
void factory_scan() {
  CommsSerial.println(
      "factory_scan not yet implemented - NAND spare-area read support required");
}

void begin() {
  BPT::begin();

  CommandRouter::add(
      cmd_log_arm,
      "log_arm");

  CommandRouter::add(
      dump_flash,
      "dump_flash",
      "use this with logging/dump.py (not yet written)");

  CommandRouter::add(
      factory_scan,
      "factory_scan",
      "TODO - requires NAND spare-area read support");
}

} // namespace Logging