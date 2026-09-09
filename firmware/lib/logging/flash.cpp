#include "flash.h"
#include "flash_defs.h"

#include "CommsSerial.h"
#include <Arduino.h>
#include <stm32h747xx.h>
#include <string.h>

#define FREQ_MAX 50'000'000
#define HAL_TIMEOUT 10000

// Pin mapping confirmed against the EC schematic (MCU.SchDoc, STM32H747BIT6) on 2026-09-09:
//   PB2  -> QUADSPI_CLK      (AF9)
//   PD11 -> QUADSPI_BK1_IO0  (AF9)
//   PD12 -> QUADSPI_BK1_IO1  (AF9)
//   PF7  -> QUADSPI_BK1_IO2  (AF9)
//   PD13 -> QUADSPI_BK1_IO3  (AF9)
//   PG6  -> QUADSPI_BK1_NCS  (AF10)
// This mirrors ASTRA's flash.cpp in hardcoding pins directly in the MSP init rather than
// going through ec_pins.h, since HAL_QSPI_MspInit is a one-time, low-level peripheral
// binding rather than a general-purpose GPIO the rest of the codebase reconfigures.
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi) {
  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_QSPI_FORCE_RESET();
  __HAL_RCC_QSPI_RELEASE_RESET();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init{};
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio_init.Pin = GPIO_PIN_2;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  gpio_init.Pin = GPIO_PIN_7;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOF, &gpio_init);

  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOG, &gpio_init);
}

namespace Flash {

QSPI_HandleTypeDef hqspi;

// True once PROGRAM LOAD has reset the cache to 0xFF for the page currently being
// assembled. Needed because write_to_cache() may be called several times (id byte,
// then struct bytes) before program() flushes the page - only the first of those
// calls should reset the rest of the cache, or later calls would stomp earlier ones.
bool cache_loaded = false;

bool exec_command(uint8_t instruction, uint32_t data_len = 0, uint32_t addr = UINT32_MAX, uint8_t addr_size_bits = 24, unsigned int dummy = 0, bool quad_data = false) {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = instruction;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.DummyCycles = dummy;
  cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (addr != UINT32_MAX) {
    cmd.Address = addr;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = (addr_size_bits == 8) ? QSPI_ADDRESS_8_BITS : (addr_size_bits == 16) ? QSPI_ADDRESS_16_BITS : QSPI_ADDRESS_24_BITS;
  }

  if (data_len) {
    cmd.NbData = data_len;
    cmd.DataMode = quad_data ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  }

  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}

bool receive(uint8_t instruction, void *data, uint32_t len, uint32_t addr = UINT32_MAX, uint8_t addr_size_bits = 24, unsigned int dummy = 0, bool quad_data = false) {
  if (!exec_command(instruction, len, addr, addr_size_bits, dummy, quad_data))
    return false;
  return HAL_QSPI_Receive(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

bool transmit(uint8_t instruction, void *data, uint32_t len, uint32_t addr = UINT32_MAX, uint8_t addr_size_bits = 24, bool quad_data = false) {
  if (!exec_command(instruction, len, addr, addr_size_bits, 0, quad_data))
    return false;
  return HAL_QSPI_Transmit(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

// GET FEATURES / SET FEATURES address the target register with a 1-byte address, not a
// memory-array address.
bool get_feature(uint8_t feature_addr, uint8_t *out) {
  return receive(CMD_NAND_GET_FEATURES, out, 1, feature_addr, 8);
}

bool set_feature(uint8_t feature_addr, uint8_t value) {
  return transmit(CMD_NAND_SET_FEATURES, &value, 1, feature_addr, 8);
}

flash_error_t read_status(flash_status_a_t *status_a, flash_status_b_t *status_b) {
  uint8_t raw;

  if (status_a) {
    if (!get_feature(FEATURE_ADDR_STATUS_A, &raw))
      return FLASH_ERROR_FAIL;
    memcpy(status_a, &raw, sizeof(raw));
  }

  if (status_b) {
    if (!get_feature(FEATURE_ADDR_STATUS_B, &raw))
      return FLASH_ERROR_FAIL;
    memcpy(status_b, &raw, sizeof(raw));
  }

  return FLASH_ERROR_SUCCESS;
}

// Polls GET FEATURES (status_a) until OIP (operation in progress) clears, or timeout_us elapses.
flash_error_t wait_until_ready(uint32_t timeout_us) {
  flash_status_a_t status{};
  uint32_t start = micros();

  do {
    if (!get_feature(FEATURE_ADDR_STATUS_A, reinterpret_cast<uint8_t *>(&status)))
      return FLASH_ERROR_FAIL;

    if (!status.OIP)
      return FLASH_ERROR_SUCCESS;

    if (micros() - start > timeout_us)
      return FLASH_ERROR_TIMED_OUT;
  } while (true);
}

flash_error_t read_page(uint32_t addr, uint8_t *out) {
  // Page Read: array -> cache.
  if (!exec_command(CMD_NAND_PAGE_READ, 0, addr, 24))
    return FLASH_ERROR_FAIL;

  flash_error_t err = wait_until_ready(50000); // TODO - confirm max tR from datasheet
  if (err != FLASH_ERROR_SUCCESS)
    return err;

  // Read From Cache: cache -> host, quad data lines, starting at column 0.
  // TODO - confirm dummy cycle count for the quad read opcode against the datasheet.
  if (!receive(CMD_NAND_READ_FROM_CACHE_QUAD, out, FLASH_PAGE_SIZE, 0, 16, 8, true))
    return FLASH_ERROR_FAIL;

  flash_status_a_t status_a{};
  if (read_status(&status_a, nullptr) != FLASH_ERROR_SUCCESS)
    return FLASH_ERROR_FAIL;

  // TODO - confirm which ECCS value(s) mean "uncorrectable" vs. "corrected" against the
  // datasheet (DS p.42/46). Treating any nonzero ECCS as a failure is conservative for now.
  if (status_a.ECCS != 0)
    return FLASH_ERROR_FAIL;

  return FLASH_ERROR_SUCCESS;
}

flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len) {
  if (col_addr + len > FLASH_PAGE_SIZE)
    return FLASH_ERROR_FAIL;

  // The first write into a fresh cache uses PROGRAM LOAD, which resets the rest of the
  // cache to 0xFF - so unwritten bytes in a partially-filled page read back as erased
  // rather than leftover data from whatever was cached before. Later writes in the same
  // page cycle use PROGRAM LOAD RANDOM DATA so they don't clobber earlier writes.
  uint8_t cmd = cache_loaded ? CMD_NAND_PROGRAM_LOAD_RANDOM : CMD_NAND_PROGRAM_LOAD;

  if (!transmit(cmd, data, len, col_addr, 16))
    return FLASH_ERROR_FAIL;

  cache_loaded = true;
  return FLASH_ERROR_SUCCESS;
}

flash_error_t program(uint32_t addr, uint32_t timeout_us) {
  flash_error_t err = wait_until_ready(timeout_us);
  if (err != FLASH_ERROR_SUCCESS)
    return err;

  if (!exec_command(CMD_NAND_WRITE_ENABLE))
    return FLASH_ERROR_FAIL;

  if (!exec_command(CMD_NAND_PROGRAM_EXECUTE, 0, addr, 24))
    return FLASH_ERROR_FAIL;

  err = wait_until_ready(timeout_us);
  cache_loaded = false; // next write_to_cache() call starts a fresh cache, regardless of outcome

  if (err != FLASH_ERROR_SUCCESS)
    return err;

  flash_status_a_t status_a{};
  if (read_status(&status_a, nullptr) != FLASH_ERROR_SUCCESS)
    return FLASH_ERROR_FAIL;

  if (status_a.P_FAIL)
    return FLASH_ERROR_FAIL; // caller (Logging) is expected to mark this page/block bad

  return FLASH_ERROR_SUCCESS;
}

flash_error_t erase_block(uint32_t addr, uint32_t timeout_us) {
  flash_error_t err = wait_until_ready(timeout_us);
  if (err != FLASH_ERROR_SUCCESS)
    return err;

  if (!exec_command(CMD_NAND_WRITE_ENABLE))
    return FLASH_ERROR_FAIL;

  if (!exec_command(CMD_NAND_BLOCK_ERASE, 0, addr, 24))
    return FLASH_ERROR_FAIL;

  err = wait_until_ready(timeout_us);
  if (err != FLASH_ERROR_SUCCESS)
    return err;

  flash_status_a_t status_a{};
  if (read_status(&status_a, nullptr) != FLASH_ERROR_SUCCESS)
    return FLASH_ERROR_FAIL;

  if (status_a.E_FAIL)
    return FLASH_ERROR_FAIL; // caller (BPT) is expected to mark this block bad

  return FLASH_ERROR_SUCCESS;
}

bool init_qspi_peripheral() {
  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX;
  if (hqspi.Init.ClockPrescaler > 255)
    hqspi.Init.ClockPrescaler = 255;

  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  hqspi.Init.FlashSize = 23; // TODO - set to log2(chip size in bytes) - 1 once the part is confirmed
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;

  return HAL_QSPI_Init(&hqspi) == HAL_OK;
}

bool reset_and_check_id() {
  if (!exec_command(CMD_NAND_RESET))
    return false;

  delay(2); // TODO - confirm reset recovery time against the datasheet

  uint8_t id[2]; // TODO - confirm ID response length/format against the datasheet
  if (!receive(CMD_NAND_READ_ID, id, sizeof(id), 0x00, 8, 8))
    return false;

  CommsSerial.print("NAND JEDEC ID: ");
  for (size_t i = 0; i < sizeof(id); ++i) {
    CommsSerial.print(id[i], HEX);
    CommsSerial.print(' ');
  }
  CommsSerial.println();
  // TODO - compare against the known-good ID once the part is finalized.

  return true;
}

bool enable_quad_mode() {
  uint8_t config;
  if (!get_feature(FEATURE_ADDR_CONFIG, &config))
    return false;

  config |= CONFIG_QE_BIT;
  return set_feature(FEATURE_ADDR_CONFIG, config);
}

void begin() {
  if (!init_qspi_peripheral()) {
    while (1) {
      CommsSerial.println("QSPI Init Failed");
      delay(1000);
    }
  }

  if (!reset_and_check_id()) {
    while (1) {
      CommsSerial.println("NAND reset/ID check failed");
      delay(1000);
    }
  }

  if (!enable_quad_mode()) {
    while (1) {
      CommsSerial.println("NAND enable quad mode failed");
      delay(1000);
    }
  }

  cache_loaded = false;

  CommsSerial.println("NAND flash driver ready.");
}

}; // namespace Flash
