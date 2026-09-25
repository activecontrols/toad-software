/**
 * @file flash.cpp
 * @brief NAND flash implementation for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include "CommsSerial.h"
#include "flash.h"
#include "flash_defs.h"
#include "stm32h7xx_hal_qspi.h"

#include <Arduino.h>
#include <cstdint>
#include <stm32h747xx.h>
#include <string.h>

namespace Flash {

QSPI_HandleTypeDef hqspi;

bool cache_loaded = false;

#define QSPI_CMD_OR_RETURN(cmd_ptr, fail_ret)                     \
  do {                                                            \
    if (HAL_QSPI_Command(&hqspi, (cmd_ptr), HAL_TIMEOUT) != HAL_OK) \
      return (fail_ret);                                          \
  } while (0)

// Receives a QSPI data phase; returns fail_ret from the calling function if it fails.
#define QSPI_RECV_OR_RETURN(data_ptr, fail_ret)                     \
  do {                                                              \
    if (HAL_QSPI_Receive(&hqspi, (data_ptr), HAL_TIMEOUT) != HAL_OK) \
      return (fail_ret);                                            \
  } while (0)

typedef struct {
  uint32_t opcode;
  uint32_t addr = UINT32_MAX;              // UINT32_MAX = no address phase
  uint32_t addr_size = QSPI_ADDRESS_8_BITS;
  uint32_t data_size = 0;                  // 0 = no data phase
  uint32_t dummy_cycles = 0;
  uint32_t data_lines = QSPI_DATA_1_LINE;
} cmd_spec;

void build_cmd(QSPI_CommandTypeDef *cmd, cmd_spec spec) {
  memset(cmd, 0, sizeof(*cmd));

  cmd->Instruction      = spec.opcode;
  cmd->InstructionMode  = QSPI_INSTRUCTION_1_LINE; // during instr phase, use 1 spi line
  cmd->DummyCycles      = spec.dummy_cycles;

  // Some commands have no address phase at all
  if (spec.addr != UINT32_MAX) {
    cmd->Address        = spec.addr;
    cmd->AddressSize    = spec.addr_size;
    cmd->AddressMode    = QSPI_ADDRESS_1_LINE; // during address phase, use 1 spi line
  }

  if (spec.data_size != 0) {
    cmd->DataMode       = spec.data_lines;
    cmd->NbData         = spec.data_size;
  }
}

bool get_status_a(flash_status_a_t *data_out) {
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_GET_FEATURES, .addr = FEATURE_ADDR_STATUS_A, .data_size = 1});
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t data;
  QSPI_RECV_OR_RETURN(&data, false);

  memcpy(data_out, &data, sizeof(data));
  return true;
};


bool get_status_b(flash_status_b_t *data_out) {
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_GET_FEATURES, .addr = FEATURE_ADDR_STATUS_B, .data_size = 1});
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t data;
  QSPI_RECV_OR_RETURN(&data, false);

  memcpy(data_out, &data, sizeof(data));
  return true;
};

flash_error_t wait_until_ready(uint32_t timeout) {
  uint32_t start_time = micros();

  while (true) {
    flash_status_a_t data;
    if (!get_status_a(&data)) {
      return FLASH_FAIL;
    }

    // if operation no longer in progress
    if (data.OIP == 0) {
      return FLASH_SUCCESS;
    }

    if (micros() - start_time > timeout) {
      return FLASH_TIMED_OUT;
    }
  }
}

flash_error_t read_page(uint32_t addr, uint8_t *data_out) {
  // load page into cache
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_PAGE_READ, .addr = addr, .addr_size = QSPI_ADDRESS_24_BITS});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for page to arrive in cache
  flash_error_t err = wait_until_ready(60);
  if (err != FLASH_SUCCESS) {
    return err;
  }

  // tell cache to send data
  build_cmd(&cmd, {.opcode = CMD_NAND_READ_FROM_CACHE_QUAD, .addr = 0, .addr_size = QSPI_ADDRESS_16_BITS, .data_size = NAND_PAGE_SIZE, .dummy_cycles = 8, .data_lines = QSPI_DATA_4_LINES});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // receive data pointer from cache across spi
  QSPI_RECV_OR_RETURN(data_out, FLASH_FAIL);

  // load status register
  flash_status_a_t status_a;
  memset(&status_a, 0, sizeof(status_a));
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  // check whether read was ok
  if (status_a.ECCS != 0) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

flash_error_t read_spare(uint32_t addr, uint8_t *data_out, size_t len) {
  if (len > NAND_SPARE_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  // load page into cache
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_PAGE_READ, .addr = addr, .addr_size = QSPI_ADDRESS_24_BITS});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for page to arrive in cache
  flash_error_t err = wait_until_ready(60);
  if (err != FLASH_SUCCESS) {
    return err;
  }

  // tell cache to send data, starting at the spare area's column offset
  build_cmd(&cmd, {.opcode = CMD_NAND_READ_FROM_CACHE_QUAD, .addr = NAND_PAGE_SIZE, .addr_size = QSPI_ADDRESS_16_BITS, .data_size = static_cast<uint32_t>(len), .dummy_cycles = 8, .data_lines = QSPI_DATA_4_LINES});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // receive spare bytes from cache across spi
  QSPI_RECV_OR_RETURN(data_out, FLASH_FAIL);

  return FLASH_SUCCESS;
}

flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len) {
  // does cache page have enough space?
  if (col_addr + len > NAND_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  // The first write to cache needs to use Program Load to clear cache
  // and afterwards need to use Load Random to not overwrite content
  uint32_t nand_cmd = cache_loaded ? CMD_NAND_PROGRAM_LOAD_RANDOM : CMD_NAND_PROGRAM_LOAD;

  // load data into cache
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = nand_cmd, .addr = col_addr, .addr_size = QSPI_ADDRESS_16_BITS, .data_size = len});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // send data across spi into cache
  if (HAL_QSPI_Transmit(&hqspi, data, HAL_TIMEOUT) != HAL_OK) {
    return FLASH_FAIL;
  }

  cache_loaded = true;
  return FLASH_SUCCESS;
}

flash_error_t program(uint32_t addr) {
  // chip can lose commands if it is busy when instruction occurs
  // ensure worst case timeout is waited until chip free
  flash_error_t err = wait_until_ready(WORST_CASE_PROGRAM_TIMEOUT);
  if (err != FLASH_SUCCESS)
    return err;

  // write enable
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_WRITE_ENABLE});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // commit cache to memory
  build_cmd(&cmd, {.opcode = CMD_NAND_PROGRAM_EXECUTE, .addr = addr, .addr_size = QSPI_ADDRESS_24_BITS});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for cache to be written
  err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  cache_loaded = false;

  if (err != FLASH_SUCCESS) {
    return err;
  }

  // ensure successful write
  flash_status_a_t status_a;
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  if (status_a.P_FAIL) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

flash_error_t erase_block(uint32_t addr) {
  // ensure chip is done with any previous erases
  flash_error_t err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  if (err != FLASH_SUCCESS)
    return err;

  // write enable
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_WRITE_ENABLE});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // erase block
  build_cmd(&cmd, {.opcode = CMD_NAND_BLOCK_ERASE, .addr = addr, .addr_size = QSPI_ADDRESS_24_BITS});
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for erase to finish
  err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  if (err != FLASH_SUCCESS) {
    return err;
  }

  // ensure erase was successful
  flash_status_a_t status_a;
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  if (status_a.E_FAIL) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

void init_qspi_gpio() {
  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init{};
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // CLK, IO0, IO1, IO3 - all AF9
  gpio_init.Alternate = GPIO_AF9_QUADSPI;

  gpio_init.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  gpio_init.Pin = GPIO_PIN_7; // PF7 - IO2
  HAL_GPIO_Init(GPIOF, &gpio_init);

  // NCS - AF10
  gpio_init.Alternate = GPIO_AF10_QUADSPI;
  gpio_init.Pin = GPIO_PIN_6; // PG6 - NCS
  HAL_GPIO_Init(GPIOG, &gpio_init);
}

bool disable_block_protection() {
  QSPI_CommandTypeDef cmd;
  build_cmd(&cmd, {.opcode = CMD_NAND_SET_FEATURES, .addr = FEATURE_ADDR_BLOCK_LOCK, .data_size = 1});
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t unlock = 0x00;
  return HAL_QSPI_Transmit(&hqspi, &unlock, HAL_TIMEOUT) == HAL_OK;
}

bool init_qspi_peripheral() {
  init_qspi_gpio();

  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX;
  if (hqspi.Init.ClockPrescaler > 255)
    hqspi.Init.ClockPrescaler = 255;

  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  hqspi.Init.FlashSize = 26;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;

  return HAL_QSPI_Init(&hqspi) == HAL_OK;
}

bool reset_and_check_id() {
  QSPI_CommandTypeDef cmd;

  // reset
  build_cmd(&cmd, {.opcode = CMD_NAND_RESET});
  QSPI_CMD_OR_RETURN(&cmd, false);

  // wait for reset to happen
  delayMicroseconds(500); 

  // read id - opcode, 1 dummy byte, then MID/DID data bytes
  build_cmd(&cmd, {.opcode = CMD_NAND_READ_ID, .data_size = 2, .dummy_cycles = 8});
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t id[2];
  QSPI_RECV_OR_RETURN(id, false);

  // MID & DID ID values for GD5F1GQ5UExxG
  if (id[0] != 0xC8 || id[1] != 0x51) {
    CommsSerial.print("Bad Chip ID Values");
    return false;
  }

  return true;
}

bool enable_quad_mode() {
  QSPI_CommandTypeDef cmd;

  // tell registers to send config
  build_cmd(&cmd, {.opcode = CMD_NAND_GET_FEATURES, .addr = FEATURE_ADDR_CONFIG, .data_size = 1});
  QSPI_CMD_OR_RETURN(&cmd, false);

  // receive config
  uint8_t config;
  QSPI_RECV_OR_RETURN(&config, false);

  config |= CONFIG_QE_BIT;

  // write updated config back
  build_cmd(&cmd, {.opcode = CMD_NAND_SET_FEATURES, .addr = FEATURE_ADDR_CONFIG, .data_size = 1});
  QSPI_CMD_OR_RETURN(&cmd, false);

  return HAL_QSPI_Transmit(&hqspi, &config, HAL_TIMEOUT) == HAL_OK;
}

bool begin() {
  if (!init_qspi_peripheral()) {
    CommsSerial.println("QSPI Init Failed");
    return false;
  }

  if (!reset_and_check_id()) {
    CommsSerial.println("NAND reset/ID check failed");
    return false;
  }

  if (!disable_block_protection()) {
    CommsSerial.println("NAND disable block protection failed");
    return false;
  }

  if (!enable_quad_mode()) {
    CommsSerial.println("NAND enable quad mode failed");
    return false;
  }

  cache_loaded = false;

  CommsSerial.println("NAND flash driver ready");

  return true;
}

}; // namespace Flash
