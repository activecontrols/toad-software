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

// Advances write_page_addr to the next good page, wrapping past any bad ones.
// Returns false if the NAND is full.
bool advance_to_next_good_page() {
  write_page_addr++;
  while (write_page_addr < NAND_NUM_PAGES && !BPT::is_page_good(write_page_addr)) {
    write_page_addr++;
  }
  write_col_addr = 0;
  return write_page_addr < NAND_NUM_PAGES;
}

void flush_page() {
  if (write_col_addr == 0)
    return; // nothing written into this page's cache yet

  if (Flash::program(write_page_addr) != FLASH_ERROR_SUCCESS) {
    BPT::mark_page_bad(write_page_addr);
  }
}

void write(uint8_t struct_id, uint8_t *data, uint32_t len) {
  if (!log_enabled)
    return;

  uint32_t entry_len = 1 + len; // id byte + struct data

  if (entry_len > FLASH_PAGE_SIZE) {
    CommsSerial.println("Logging::write - struct too large to fit in one page, dropped");
    return;
  }

  // Never split an entry across a page boundary, so a reader can always trust that an id
  // byte is immediately followed by that struct's full data within the same page.
  if (write_col_addr + entry_len > FLASH_PAGE_SIZE) {
    flush_page();
    if (!advance_to_next_good_page()) {
      CommsSerial.println("Logging::write - out of flash, disarming");
      log_enabled = false;
      return;
    }
  }

  Flash::write_to_cache(write_col_addr, &struct_id, 1);
  write_col_addr += 1;

  Flash::write_to_cache(write_col_addr, data, len);
  write_col_addr += len;
}

void complete() {
  if (!log_enabled)
    return;

  flush_page();
  log_enabled = false;
}

flash_error_t read_page(uint32_t addr, uint32_t *data) {
  if (!BPT::is_page_good(addr))
    return FLASH_ERROR_FAIL;

  flash_error_t err = Flash::read_page(addr, reinterpret_cast<uint8_t *>(data));
  if (err != FLASH_ERROR_SUCCESS) {
    BPT::mark_page_bad(addr); // ECC failure - flag this page/block bad for next time
  }

  return err;
}

// Requires a random confirmation key (same pattern as ASTRA's cmd_log_arm) before erasing
// every currently-good block and enabling writes. This is the gate that's supposed to stop
// anyone from accidentally programming the NAND before a factory bad-block scan has run -
// see the TODO on factory_scan() below.
void cmd_log_arm() {
  if (log_enabled) {
    CommsSerial.println("Log is already enabled.");
    return;
  }

  const int key_min = 1000;
  const int key_max = 10000;
  srand(millis());
  int key = rand() % (key_max - key_min) + key_min;

  CommsSerial.println("WARNING: Proceeding will erase all data currently on the NAND.");
  CommsSerial.printf("Enter %d to proceed: \n", key);

  char *res = CommsSerial.readline();
  if (atoi(res) != key) {
    CommsSerial.println("Incorrect key entered.");
    return;
  }

  CommsSerial.println("Erasing all good blocks. This may take a while.");

  for (uint32_t block = 0; block < NAND_NUM_BLOCKS; ++block) {
    uint32_t page_addr = block * NAND_PAGES_PER_BLOCK;
    if (!BPT::is_page_good(page_addr))
      continue;

    if (Flash::erase_block(page_addr) != FLASH_ERROR_SUCCESS) {
      BPT::mark_page_bad(page_addr);
    }
  }

  CommsSerial.println("Erase complete. Logging enabled.");

  write_page_addr = 0;
  while (write_page_addr < NAND_NUM_PAGES && !BPT::is_page_good(write_page_addr)) {
    write_page_addr++;
  }
  write_col_addr = 0;
  log_enabled = true;
}

// Streams the log out page-by-page over USB serial, mirroring ASTRA's send_flash_over_serial()
// protocol (2-byte additive checksum, 'c'/'k' handshake, all-0xFF page = end of log) - see
// firmware/lib/logging/dump.py (TODO - still needs writing, adapted from ASTRA's dump.py, plus
// the struct/type-prefix decoder into CSV that doesn't seem to exist on ASTRA either).
//
// Walking page indices in order and skipping whatever BPT currently marks bad reproduces the
// exact sequence write() used while logging, since write() does the same walk/skip - as long
// as BPT hasn't changed between logging and downloading.
void dump_flash() {
  static uint8_t page_buf[FLASH_PAGE_SIZE];

  while (USB_CommsSerial.available()) {
    USB_CommsSerial.read();
  }

  for (uint32_t addr = 0; addr < NAND_NUM_PAGES; ++addr) {
    if (!BPT::is_page_good(addr))
      continue;

    if (Flash::read_page(addr, page_buf) != FLASH_ERROR_SUCCESS) {
      BPT::mark_page_bad(addr);
      continue;
    }

    bool all_ff = true;
    uint8_t cs_a = 0, cs_b = 0;
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
      cs_a += page_buf[i];
      cs_b += cs_a;
      if (page_buf[i] != 0xFF)
        all_ff = false;
    }

    if (all_ff) {
      USB_CommsSerial.write('k');
      return;
    }

    USB_CommsSerial.write(page_buf, sizeof(page_buf));
    USB_CommsSerial.write(cs_a);
    USB_CommsSerial.write(cs_b);
    USB_CommsSerial.write('c');

    int c;
    uint32_t wait_start = millis();
    do {
      c = USB_CommsSerial.read();
      if (millis() - wait_start > 5000) {
        CommsSerial.println("Error: flash dump timed out");
        return;
      }
    } while (!(c == 'k' || c == 'c'));

    if (c == 'k')
      return;
  }

  USB_CommsSerial.write('k');
}

// TODO: scanning for factory-bad blocks needs to read each block's OOB/spare area (the
// factory bad-block marker lives past the FLASH_PAGE_SIZE=2048 ECC-covered region that
// Flash::read_page() exposes), which isn't part of the Flash interface yet. Needs a new
// low-level primitive (e.g. a raw, non-ECC-checked read at a column offset >= 2048) once
// the OOB layout is confirmed against the final NAND datasheet.
//
// This MUST run (and BPT get built from it) before any write ever touches the NAND -
// that's the whole reason cmd_log_arm() requires a confirmation key first.
void factory_scan() {
  CommsSerial.println("factory_scan not yet implemented - see TODO in logging.cpp");
}

void begin() {
  BPT::begin();

  CommandRouter::add(cmd_log_arm, "log_arm");
  CommandRouter::add(dump_flash, "dump_flash", "use this with logging/dump.py (not yet written)");
  CommandRouter::add(factory_scan, "factory_scan", "TODO - not yet implemented, see logging.cpp");
}

} // namespace Logging
