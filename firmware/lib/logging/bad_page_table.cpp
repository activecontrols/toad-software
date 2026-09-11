#include "bad_page_table.h"
#include "flash_defs.h"

#include "CommsSerial.h"
#include <Arduino.h>
#include <string.h>

#define BPT_MAGIC 0x50425401u

// The BPT stores one bit per NAND block. A cleared bit means the block is good and a
// set bit means the block has been marked bad.
#define BITMAP_SIZE_BYTES ((NAND_NUM_BLOCKS + 7u) / 8u)

// Stored separately from the bitmap so that an initialized BPT can be distinguished
// from one that has actually completed the factory bad-block scan.
#define BPT_FLAG_FACTORY_SCAN_COMPLETE (1u << 0)

struct __attribute__((packed)) bpt_header_t {
  uint32_t magic;
  uint32_t flags;
};

#define BPT_TABLE_SIZE_BYTES (sizeof(bpt_header_t) + BITMAP_SIZE_BYTES)

// STM32H7 internal flash is programmed one 256-bit flash word (32 bytes) at a time,
// so round the table size up to the next complete flash word.
#define FLASH_WORD_SIZE 32u
#define BPT_WRITE_SIZE \
  (((BPT_TABLE_SIZE_BYTES + FLASH_WORD_SIZE - 1u) / FLASH_WORD_SIZE) * FLASH_WORD_SIZE)

// Provided by the linker script. This region must be reserved so that normal program
// data is never placed in the sector used to store the BPT.
extern uint32_t _bpt_table_start;

namespace BPT {

static uint32_t bpt_base_addr = 0;

// Returns a pointer to the header stored at the beginning of the reserved BPT region.
static const bpt_header_t *header_ptr() {
  return reinterpret_cast<const bpt_header_t *>(bpt_base_addr);
}

// The bitmap immediately follows the BPT header in internal flash.
static const uint8_t *bitmap_ptr() {
  return reinterpret_cast<const uint8_t *>(
      bpt_base_addr + sizeof(bpt_header_t));
}

static bool valid_page_addr(uint32_t page_addr) {
  return page_addr < NAND_NUM_PAGES;
}

// Any change to the BPT requires erasing and rewriting its internal flash sector.
// The voltage range still needs to be confirmed for the actual board configuration.
static bool erase_bpt_sector() {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase_init{};
  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.Banks = FLASH_BANK_2;
  erase_init.Sector = FLASH_SECTOR_0;
  erase_init.NbSectors = 1;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // TODO - confirm for this board

  uint32_t sector_error = 0;
  HAL_StatusTypeDef status =
      HAL_FLASHEx_Erase(&erase_init, &sector_error);

  HAL_FLASH_Lock();

  return status == HAL_OK;
}

// Programs one 32-byte flash word into STM32 internal flash.
static bool program_flash_word(uint32_t address, const uint8_t *data) {
  HAL_FLASH_Unlock();

  HAL_StatusTypeDef status =
      HAL_FLASH_Program(
          FLASH_TYPEPROGRAM_FLASHWORD,
          address,
          reinterpret_cast<uint32_t>(data));

  HAL_FLASH_Lock();

  return status == HAL_OK;
}

// Erases the BPT sector and rewrites the header and bitmap in one pass. The buffer
// is padded with 0xFF so its size is an exact multiple of the STM32 flash word size.
static bool write_table(const bpt_header_t &header, const uint8_t *bitmap) {
  if (!erase_bpt_sector())
    return false;

  alignas(32) uint8_t buf[BPT_WRITE_SIZE];
  memset(buf, 0xFF, sizeof(buf));

  memcpy(buf, &header, sizeof(header));
  memcpy(buf + sizeof(header), bitmap, BITMAP_SIZE_BYTES);

  for (uint32_t offset = 0;
       offset < sizeof(buf);
       offset += FLASH_WORD_SIZE) {

    if (!program_flash_word(
            bpt_base_addr + offset,
            buf + offset)) {
      return false;
    }
  }

  return true;
}

bool is_page_good(uint32_t page_addr) {
  if (!valid_page_addr(page_addr))
    return false;

  // NAND bad blocks are tracked at block granularity. Convert the page index into
  // its containing block before looking up its bit in the table.
  uint32_t block = page_addr / NAND_PAGES_PER_BLOCK;

  const uint8_t *bitmap = bitmap_ptr();
  uint8_t mask =
      static_cast<uint8_t>(1u << (block % 8u));

  bool bad = (bitmap[block / 8u] & mask) != 0;

  return !bad;
}

void mark_page_bad(uint32_t page_addr) {
  if (!valid_page_addr(page_addr)) {
    CommsSerial.println("BPT: invalid page address");
    return;
  }

  if (!is_page_good(page_addr))
    return; // containing block is already marked bad

  uint32_t block = page_addr / NAND_PAGES_PER_BLOCK;

  // Make RAM copies of the current table before erasing the sector. Only the bit
  // corresponding to the newly bad block is changed.
  bpt_header_t header;
  memcpy(
      &header,
      reinterpret_cast<const void *>(bpt_base_addr),
      sizeof(header));

  uint8_t bitmap[BITMAP_SIZE_BYTES];
  memcpy(bitmap, bitmap_ptr(), sizeof(bitmap));

  bitmap[block / 8u] |=
      static_cast<uint8_t>(1u << (block % 8u));

  if (!write_table(header, bitmap)) {
    CommsSerial.println("BPT: failed to persist newly-marked bad block");
  }
}

bool factory_scan_complete() {
  const bpt_header_t *header = header_ptr();

  if (header->magic != BPT_MAGIC)
    return false;

  return (header->flags &
          BPT_FLAG_FACTORY_SCAN_COMPLETE) != 0;
}

bool mark_factory_scan_complete() {
  if (factory_scan_complete())
    return true;

  // Preserve the current bitmap and update only the factory scan flag.
  bpt_header_t header;
  memcpy(
      &header,
      reinterpret_cast<const void *>(bpt_base_addr),
      sizeof(header));

  uint8_t bitmap[BITMAP_SIZE_BYTES];
  memcpy(bitmap, bitmap_ptr(), sizeof(bitmap));

  header.flags |= BPT_FLAG_FACTORY_SCAN_COMPLETE;

  if (!write_table(header, bitmap)) {
    CommsSerial.println("BPT: failed to persist factory scan flag");
    return false;
  }

  return true;
}

void begin() {
  bpt_base_addr =
      reinterpret_cast<uint32_t>(&_bpt_table_start);

  bpt_header_t header;
  memcpy(
      &header,
      reinterpret_cast<const void *>(bpt_base_addr),
      sizeof(header));

  if (header.magic == BPT_MAGIC)
    return; // table already initialized

  CommsSerial.println("Bad Page Table is not initialized.");
  CommsSerial.println(
      "This will initialize BPT storage, but will not scan the NAND for factory-bad blocks.");
  CommsSerial.print("Type 'yes' to initialize the table: ");

  char *response = CommsSerial.readline();

  if (strcmp(response, "yes") != 0) {
    while (1) {
      CommsSerial.println("BPT not initialized - halting.");
      delay(1000);
    }
  }

  // Start with no blocks marked bad. The factory-scan flag remains clear, so NAND
  // erase/program operations must stay locked until the factory scan finishes.
  uint8_t bitmap[BITMAP_SIZE_BYTES];
  memset(bitmap, 0x00, sizeof(bitmap));

  bpt_header_t new_header{};
  new_header.magic = BPT_MAGIC;
  new_header.flags = 0;

  if (!write_table(new_header, bitmap)) {
    while (1) {
      CommsSerial.println("Failed to initialize BPT - halting.");
      delay(1000);
    }
  }

  CommsSerial.println("BPT initialized. Factory scan still required.");
}

} // namespace BPT