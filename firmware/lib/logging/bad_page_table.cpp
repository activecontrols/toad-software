#include "bad_page_table.h"
#include "flash_defs.h"

#include "CommsSerial.h"
#include <Arduino.h>
#include <string.h>

#define BPT_MAGIC 0x50425401u // 'PBT' + version 1 - arbitrary, just needs to be unlikely to appear in erased flash

#define BITMAP_SIZE_BYTES ((NAND_NUM_BLOCKS + 7) / 8)
#define BPT_TABLE_SIZE_BYTES (sizeof(uint32_t) + BITMAP_SIZE_BYTES)
// STM32H7 internal flash program granularity is one 256-bit "flash word" (32 bytes) at a time.
#define FLASH_WORD_SIZE 32u
#define BPT_WRITE_SIZE (((BPT_TABLE_SIZE_BYTES + FLASH_WORD_SIZE - 1) / FLASH_WORD_SIZE) * FLASH_WORD_SIZE)

// Provided by the linker (ldscript.ld) - base of the reserved Bank 2, Sector 0 region.
extern uint32_t _bpt_table_start;

namespace BPT {

uint32_t bpt_base_addr;

// STM32H7's flash is ECC-protected per flash word: once a word is programmed, it cannot be
// programmed again (even to clear further bits to 0) without erasing its sector first - unlike
// classic STM32 NOR-style flash. So any update to the table means erase-the-sector-and-rewrite-
// the-whole-thing, not a targeted bit flip.
bool erase_bpt_sector() {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase_init{};
  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.Banks = FLASH_BANK_2;
  erase_init.Sector = FLASH_SECTOR_0;
  erase_init.NbSectors = 1;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // TODO - confirm correct voltage range for this board

  uint32_t sector_error;
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

  HAL_FLASH_Lock();
  return status == HAL_OK;
}

bool program_flash_word(uint32_t address, const uint8_t *data) {
  HAL_FLASH_Unlock();
  HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, reinterpret_cast<uint32_t>(data));
  HAL_FLASH_Lock();
  return status == HAL_OK;
}

// Erases the BPT sector and rewrites the magic number + given bitmap in one pass.
bool write_table(const uint8_t *bitmap) {
  if (!erase_bpt_sector())
    return false;

  uint8_t buf[BPT_WRITE_SIZE];
  memset(buf, 0xFF, sizeof(buf));

  uint32_t magic = BPT_MAGIC;
  memcpy(buf, &magic, sizeof(magic));
  memcpy(buf + sizeof(magic), bitmap, BITMAP_SIZE_BYTES);

  for (uint32_t offset = 0; offset < sizeof(buf); offset += FLASH_WORD_SIZE) {
    if (!program_flash_word(bpt_base_addr + offset, buf + offset))
      return false;
  }

  return true;
}

bool is_page_good(uint32_t addr) {
  uint32_t block = addr / NAND_PAGES_PER_BLOCK;

  const uint8_t *bitmap = reinterpret_cast<const uint8_t *>(bpt_base_addr + sizeof(uint32_t));
  bool bad = bitmap[block / 8] & (1u << (block % 8));

  return !bad;
}

void mark_page_bad(uint32_t addr) {
  uint32_t block = addr / NAND_PAGES_PER_BLOCK;

  if (!is_page_good(addr))
    return; // already marked

  uint8_t bitmap[BITMAP_SIZE_BYTES];
  memcpy(bitmap, reinterpret_cast<const void *>(bpt_base_addr + sizeof(uint32_t)), sizeof(bitmap));
  bitmap[block / 8] |= (1u << (block % 8));

  if (!write_table(bitmap)) {
    CommsSerial.println("BPT: failed to persist newly-marked bad block!");
  }
}

void begin() {
  bpt_base_addr = reinterpret_cast<uint32_t>(&_bpt_table_start);

  uint32_t magic;
  memcpy(&magic, reinterpret_cast<const void *>(bpt_base_addr), sizeof(magic));

  if (magic == BPT_MAGIC)
    return; // table already valid

  CommsSerial.println("Bad Page Table is not initialized.");
  CommsSerial.println("This will erase the current BPT and mark every page as good.");
  CommsSerial.println("This does NOT scan the NAND for factory-bad blocks - run the factory scan separately before the first write.");
  CommsSerial.print("Type 'yes' to initialize the table: ");

  char *response = CommsSerial.readline();

  if (strcmp(response, "yes") != 0) {
    while (1) {
      CommsSerial.println("BPT not initialized - halting.");
      delay(1000);
    }
  }

  uint8_t bitmap[BITMAP_SIZE_BYTES];
  memset(bitmap, 0x00, sizeof(bitmap)); // 0 = good

  if (!write_table(bitmap)) {
    while (1) {
      CommsSerial.println("Failed to initialize BPT - halting.");
      delay(1000);
    }
  }

  CommsSerial.println("BPT initialized.");
}

} // namespace BPT
