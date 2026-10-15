/**
 * @file flash.cpp
 * @brief NAND flash implementation for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include "flash.h"
#include "CommsSerial.h"
#include "flash_defs.h"
#include "stm32h7xx_hal_qspi.h"

#include <Arduino.h>
#include <cstdint>
#include <stm32h747xx.h>
#include <string.h>

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *) {
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

  // PB2, PD11, PD12, PD13, PF7 all under AF9
  gpio_init.Alternate = GPIO_AF9_QUADSPI;
  
  gpio_init.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  gpio_init.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOF, &gpio_init);

  // PG6 is under AF10
  gpio_init.Alternate = GPIO_AF10_QUADSPI;
  gpio_init.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOG, &gpio_init);
}

namespace Flash {

namespace {

QSPI_HandleTypeDef hqspi;

// Header phase command
#define QSPI_CMD_OR_RETURN(cmd_ptr, fail_ret)                       \
  do {                                                              \
    if (HAL_QSPI_Command(&hqspi, (cmd_ptr), QSPI_TIMEOUT_MS) != HAL_OK) \
      return (fail_ret);                                            \
  } while (0)

// Data phase command
#define QSPI_TRANSMIT_OR_RETURN(data_ptr, fail_ret)                   \
  do {                                                                \
    if (HAL_QSPI_Transmit(&hqspi, (data_ptr), QSPI_TIMEOUT_MS) != HAL_OK) \
      return (fail_ret);                                              \
  } while (0)

// Receives a QSPI data phase
#define QSPI_RECV_OR_RETURN(data_ptr, fail_ret)                      \
  do {                                                               \
    if (HAL_QSPI_Receive(&hqspi, (data_ptr), QSPI_TIMEOUT_MS) != HAL_OK) \
      return (fail_ret);                                             \
  } while (0)

typedef struct {
  uint32_t opcode;
  uint32_t addr           = UINT32_MAX; // no address if max address
  uint32_t addr_size      = QSPI_ADDRESS_8_BITS;
  uint32_t data_size      = 0; // 0 = no data phase
  uint32_t dummy_cycles   = 0;
  uint32_t data_lines     = QSPI_DATA_1_LINE;
} cmd_spec;

QSPI_CommandTypeDef build_cmd(cmd_spec spec) {
  QSPI_CommandTypeDef out_cmd;
  memset(&out_cmd, 0, sizeof(out_cmd));

  out_cmd.Instruction = spec.opcode;
  // The NAND chip only recognizes instructions
  // on one line, even though the QSPI supports
  // using more for instructions
  out_cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  out_cmd.DummyCycles = spec.dummy_cycles;

  if (spec.addr != UINT32_MAX) {
    out_cmd.Address = spec.addr;
    out_cmd.AddressSize = spec.addr_size;
    out_cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  }

  if (spec.data_size != 0) {
    out_cmd.DataMode = spec.data_lines;
    out_cmd.NbData = spec.data_size;
  }

  return out_cmd;
}

// load directly from register, only need 1 Instr. CMD + 1 Receive
static bool get_feature(uint8_t feature_addr, uint8_t *data_out) {
  cmd_spec new_cmd_spec = {
    .opcode = CMD_NAND_GET_FEATURES,
    .addr = feature_addr,
    .data_size = 1,
  };
  QSPI_CommandTypeDef cmd = build_cmd(new_cmd_spec);
  QSPI_CMD_OR_RETURN(&cmd, false);
  QSPI_RECV_OR_RETURN(data_out, false);

  return true;
}

bool get_status_a(flash_status_a_t *data_out) {
  uint8_t data;
  if (!get_feature(FEATURE_ADDR_STATUS_A, &data)) {
    return false;
  }

  memcpy(data_out, &data, sizeof(data));
  return true;
};

bool get_status_b(flash_status_b_t *data_out) {
  uint8_t data;
  if (!get_feature(FEATURE_ADDR_STATUS_B, &data)) {
    return false;
  }

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
    if (!data.OIP) {
      return FLASH_READY;
    }

    if (micros() - start_time > timeout) {
      return FLASH_TIMED_OUT;
    }
  }
}

bool disable_block_protection() {
  /* page 49 of GD5F1 datasheet */

  cmd_spec disable_spec = {
    .opcode = CMD_NAND_SET_FEATURES, 
    .addr = FEATURE_ADDR_BLOCK_LOCK, 
    .data_size = 1
  };

  QSPI_CommandTypeDef cmd = build_cmd(disable_spec);
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t unlock = 0x00;
  if (HAL_QSPI_Transmit(&hqspi, &unlock, QSPI_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  // ensure write unlock succeedded
  uint8_t protection_reg;
  if (!get_feature(FEATURE_ADDR_BLOCK_LOCK, &protection_reg)) {
    return false;
  }

  if (protection_reg != unlock) {
    return false;
  }

  return true;
}

bool init_qspi_peripheral() {
  // See page 1261 of STM32H7 dm00392525 user manual
  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;

  // Clock Prescaler here is 32 bits but 
  // must be a value between 0 and 255 per
  // stm32h7xx_hal_qspi.h:53
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX;
  if (hqspi.Init.ClockPrescaler > 255)
    hqspi.Init.ClockPrescaler = 255;

  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  
  // 128 MB = 2^27 bytes
  // Flash size + 1 = # of bits to address whole array
  // Flash size = 27 - 1 = 26
  hqspi.Init.FlashSize = 26;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  
  // Dual flash defaults to disabled

  // Internally calls MspInit
  return HAL_QSPI_Init(&hqspi) == HAL_OK;
}

bool reset_and_check_id() {
  cmd_spec reset_cmd = {
    .opcode = CMD_NAND_RESET,
  };
  QSPI_CommandTypeDef cmd = build_cmd(reset_cmd);
  QSPI_CMD_OR_RETURN(&cmd, false);

  // wait for reset to happen
  delayMicroseconds(RESET_MAX_DELAY_US);

  // read ID bytes
  // read page 14 of DS-00889
  cmd_spec read_cmd = {
    .opcode = CMD_NAND_READ_ID,
    .data_size = 2,
    .dummy_cycles = 8,
  };
  cmd = build_cmd(read_cmd);
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t id[2];
  QSPI_RECV_OR_RETURN(id, false);

  if (id[0] != MANUFACTURER_ID || id[1] != DEVICE_ID) {
    CommsSerial.print("Bad Chip ID Values");
    return false;
  }

  return true;
}

bool enable_quad_mode() {
  // tell registers to send config
  cmd_spec get_config_cmd = {
    .opcode = CMD_NAND_GET_FEATURES,
    .addr = FEATURE_ADDR_CONFIG,
    .data_size = 1,
  };
  QSPI_CommandTypeDef cmd = build_cmd(get_config_cmd);
  QSPI_CMD_OR_RETURN(&cmd, false);

  // receive config
  uint8_t config;
  QSPI_RECV_OR_RETURN(&config, false);

  config |= CONFIG_QE_BIT;

  // write updated config back
  cmd_spec set_config_cmd = {
    .opcode = CMD_NAND_SET_FEATURES,
    .addr = FEATURE_ADDR_CONFIG,
    .data_size = 1,
  };
  cmd = build_cmd(set_config_cmd);
  QSPI_CMD_OR_RETURN(&cmd, false);
  
  if (HAL_QSPI_Transmit(&hqspi, &config, QSPI_TIMEOUT_MS) != HAL_OK)
    return false;

  // confirm QE bit was set
  cmd_spec read_status_cmd = {
    .opcode = CMD_NAND_GET_FEATURES,
    .addr = FEATURE_ADDR_CONFIG,
    .data_size = 1,
  };
  cmd = build_cmd(read_status_cmd);
  QSPI_CMD_OR_RETURN(&cmd, false);

  uint8_t config_check;
  QSPI_RECV_OR_RETURN(&config_check, false);

  if (!(config_check & CONFIG_QE_BIT))
    return false;
  
  return true;
}

flash_error_t read_at_col(uint32_t addr, uint32_t col, uint8_t *data_out, size_t len, bool check_ecc) {
  // Don't read past the chip's main page and spare page bounds
  if (col + len > NAND_PAGE_SIZE + NAND_SPARE_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  // load page into cache
  cmd_spec page_load_cmd = {
    .opcode = CMD_NAND_PAGE_READ, 
    .addr = addr,
    .addr_size = QSPI_ADDRESS_24_BITS,
  };
  QSPI_CommandTypeDef cmd = build_cmd(page_load_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for page to arrive in cache
  flash_error_t err = wait_until_ready(READ_ARRAY_WITH_ECC_MAX_DELAY_US);
  if (err != FLASH_READY) {
    return err;
  }

  // tell cache to send data, starting at the given column offset
  cmd_spec cache_send_cmd = {
    .opcode = CMD_NAND_READ_FROM_CACHE_QUAD,
    .addr = col,
    .addr_size = QSPI_ADDRESS_16_BITS,
    .data_size = static_cast<uint32_t>(len),
    .dummy_cycles = 8,
    .data_lines = QSPI_DATA_4_LINES
  };
  cmd = build_cmd(cache_send_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // receive data from cache across spi
  QSPI_RECV_OR_RETURN(data_out, FLASH_FAIL);

  // if we dont have to check ecc, end early
  if (!check_ecc) {
    return FLASH_SUCCESS;
  }

  // load status register
  flash_status_a_t status_a;
  memset(&status_a, 0, sizeof(status_a));
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  // check whether read was ok
  if (status_a.ECCS == UNCORRECTED_ERROR) {
    return FLASH_ECC_FAIL;
  }

  return FLASH_SUCCESS;
}

} // namespace

flash_error_t read_main(uint32_t addr, uint32_t offset, uint8_t *out, size_t len, bool check_ecc) {
  if (offset + len > NAND_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  return read_at_col(addr, offset, out, len, check_ecc);
}

flash_error_t read_spare(uint32_t addr, uint32_t offset, uint8_t *out, size_t len, bool check_ecc) {
  if (offset + len > NAND_SPARE_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  return read_at_col(addr, NAND_PAGE_SIZE + offset, out, len, check_ecc);
}

flash_error_t write_to_cache(uint32_t col, uint8_t *data, size_t len) {
  // does cache page have enough space?
  if (col + len > NAND_PAGE_SIZE) {
    return FLASH_FAIL;
  }

  // Program Load at col 0 clears the cache, Load Random keeps existing content
  uint32_t nand_cmd = col == 0 ? CMD_NAND_PROGRAM_LOAD : CMD_NAND_PROGRAM_LOAD_RANDOM;

  // load data into cache
  cmd_spec load_cmd = {
    .opcode = nand_cmd, 
    .addr = col, 
    .addr_size = QSPI_ADDRESS_16_BITS, 
    .data_size = len
  };
  QSPI_CommandTypeDef cmd = build_cmd(load_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // send data across spi into cache
  QSPI_TRANSMIT_OR_RETURN(data, FLASH_FAIL);

  return FLASH_SUCCESS;
}

flash_error_t program(uint32_t addr) {
  // chip can lose commands if it is busy when instruction occurs
  // ensure worst case timeout is waited until chip free
  flash_error_t err = wait_until_ready(WORST_CASE_PROGRAM_TIMEOUT);
  if (err != FLASH_READY)
    return err;

  cmd_spec write_enable_cmd = {
    .opcode = CMD_NAND_WRITE_ENABLE
  };
  QSPI_CommandTypeDef cmd = build_cmd(write_enable_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  cmd_spec commit_cache_cmd = {
    .opcode = CMD_NAND_PROGRAM_EXECUTE, 
    .addr = addr, 
    .addr_size = QSPI_ADDRESS_24_BITS
  };
  cmd = build_cmd(commit_cache_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for cache to be written
  err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);

  if (err != FLASH_READY) {
    return err;
  }

  // ensure successful write
  flash_status_a_t status_a;
  memset(&status_a, 0, sizeof(status_a));
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
  if (err != FLASH_READY)
    return err;

  cmd_spec enable_write_cmd = {
    .opcode = CMD_NAND_WRITE_ENABLE,
  };
  QSPI_CommandTypeDef cmd = build_cmd(enable_write_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  cmd_spec erase_cmd = {
    .opcode = CMD_NAND_BLOCK_ERASE, 
    .addr = addr, 
    .addr_size = QSPI_ADDRESS_24_BITS
  };
  cmd = build_cmd(erase_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // wait for erase to finish
  err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  if (err != FLASH_READY) {
    return err;
  }

  // ensure erase was successful
  flash_status_a_t status_a;
  memset(&status_a, 0, sizeof(status_a));
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  if (status_a.E_FAIL) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
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

  CommsSerial.println("NAND flash driver ready");

  return true;
}

}; // namespace Flash
