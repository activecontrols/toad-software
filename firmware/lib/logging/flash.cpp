/**
 * @file flash.cpp
 * @brief NAND flash implementation for GD5F1GQ5UEYIGR chip
 * @note  All pag references are to the GD5F chip datasheet
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#include "flash.h"
#include "CommsSerial.h"
#include "ec_pins.h"
#include "flash_defs.h"
#include "stm32h7xx_hal_qspi.h"

#include <Arduino.h>
#include <cstdint>
#include <stm32h747xx.h>
#include <string.h>

namespace Flash {

namespace {

QSPI_HandleTypeDef hqspi;

// Header phase command
#define QSPI_CMD_OR_RETURN(cmd_ptr, fail_ret)                           \
  do {                                                                  \
    if (HAL_QSPI_Command(&hqspi, (cmd_ptr), QSPI_TIMEOUT_MS) != HAL_OK) \
      return (fail_ret);                                                \
  } while(0)

// Data phase command
#define QSPI_TRANSMIT_OR_RETURN(data_ptr, fail_ret)                       \
  do {                                                                    \
    if (HAL_QSPI_Transmit(&hqspi, (data_ptr), QSPI_TIMEOUT_MS) != HAL_OK) \
      return (fail_ret);                                                  \
  } while(0)

// Receives a QSPI data phase
#define QSPI_RECV_OR_RETURN(data_ptr, fail_ret)                           \
  do {                                                                    \
    if (HAL_QSPI_Receive(&hqspi, (data_ptr), QSPI_TIMEOUT_MS) != HAL_OK)  \
      return (fail_ret);                                                  \
  } while(0)

#define RETURN_AND_LOG_IF_FAILED(function, msg)                      \
  do {                                                               \
    if (!(function)()) {                                             \
      CommsSerial.printf((msg));                                     \
      return false;                                                  \
    }                                                                \
  } while(0)

#define RETURN_IF_NULL(item, failure)                               \
  do {                                                              \
    if (item == NULL) {                                             \
      return failure;                                               \
    }                                                               \
  } while(0);

typedef struct {
  uint32_t opcode;
  uint32_t addr           = UINT32_MAX; // no page if max address
  uint32_t addr_size      = QSPI_ADDRESS_8_BITS;
  uint32_t data_size      = 0; // If 0, then there is no dataphase
  uint32_t dummy_cycles   = 0;
  uint32_t data_lines     = QSPI_DATA_1_LINE;
} cmd_spec;

QSPI_CommandTypeDef build_cmd(cmd_spec spec) {
  QSPI_CommandTypeDef out_cmd = {};

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

// load directly from register
bool get_feature(uint8_t feature_addr, uint8_t *data_out) {
  RETURN_IF_NULL(data_out, false);

  cmd_spec spec = {
    .opcode = CMD_NAND_GET_FEATURES,
    .addr = feature_addr,
    .data_size = 1,
  };
  QSPI_CommandTypeDef cmd = build_cmd(spec);
  QSPI_CMD_OR_RETURN(&cmd, false);
  QSPI_RECV_OR_RETURN(data_out, false);

  return true;
}

// set feature register
bool set_feature(uint8_t feature_addr, uint8_t *data) {
  RETURN_IF_NULL(data, false);

  cmd_spec spec = {
    .opcode = CMD_NAND_SET_FEATURES,
    .addr = feature_addr,
    .data_size = 1,
  };
  QSPI_CommandTypeDef cmd = build_cmd(spec);
  QSPI_CMD_OR_RETURN(&cmd, false);
  QSPI_TRANSMIT_OR_RETURN(data, false);

  return true;
}

bool get_status_a(flash_status_a_t *data_out) {
  RETURN_IF_NULL(data_out, false);

  uint8_t data;
  if (!get_feature(FEATURE_ADDR_STATUS_A, &data)) {
    return false;
  }

  memcpy(data_out, &data, sizeof(data));
  return true;
};

bool get_status_b(flash_status_b_t *data_out) {
  RETURN_IF_NULL(data_out, false);
  
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

  return FLASH_FAIL;
}

bool disable_block_protection() {
  /* page 49 of GD5F1 datasheet */

  uint8_t protection_register;
  if (!get_feature(FEATURE_ADDR_BLOCK_LOCK, &protection_register)) {
    return false;
  }

  cmd_spec disable_spec = {
    .opcode = CMD_NAND_SET_FEATURES, 
    .addr = FEATURE_ADDR_BLOCK_LOCK, 
    .data_size = 1
  };

  QSPI_CommandTypeDef cmd = build_cmd(disable_spec);
  QSPI_CMD_OR_RETURN(&cmd, false);

  // See pg. 49, zero-ing bits BP2/1/0
  protection_register &= ~BLOCK_PROTECTION_BITS;
  QSPI_TRANSMIT_OR_RETURN(&protection_register, false);

  // ensure write unlock succeedded
  uint8_t read_protection_register;
  if (!get_feature(FEATURE_ADDR_BLOCK_LOCK, &read_protection_register)) {
    return false;
  }

  if (protection_register != read_protection_register) {
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
  // Adding (FREQ_MAX - 1) to numerator acts as a ceiling operation,
  // so that we are sure clock prescalar is not truncated
  hqspi.Init.ClockPrescaler = (f_hclk + FREQ_MAX - 1) / FREQ_MAX - 1;
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
  // read page 14.
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
  uint8_t feature;
  if (!get_feature(FEATURE_ADDR_CONFIG, &feature)) {
    return false;
  }

  feature |= CONFIG_QE_BIT;

  if (!set_feature(FEATURE_ADDR_CONFIG, &feature)) {
    return false;
  }

  // Ensure write succeeded
  uint8_t config_check;
  if (!get_feature(FEATURE_ADDR_CONFIG, &config_check)) {
    return false;
  }

  if (!(config_check & CONFIG_QE_BIT))
    return false;
  
  return true;
}

bool ensure_ecc_enabled() {
  uint8_t feature;
  if (!get_feature(FEATURE_ADDR_CONFIG, &feature)) {
    return false;
  }

  // Is ECC enabled?
  bool enabled = (feature & CONFIG_ECC_EN_BIT);

  if (enabled) {
    return true;
  }

  feature |= CONFIG_ECC_EN_BIT;

  if (!set_feature(FEATURE_ADDR_CONFIG, &feature)) {
    return false;
  }

  // Ensure write succeeded

  uint8_t check_feature;

  if (!get_feature(FEATURE_ADDR_CONFIG, &check_feature)) {
    return false;
  }

  if (check_feature != feature) {
    return false;
  }

  return true;
}

flash_error_t flash_read_to_buffer(uint32_t row_addr, uint32_t col_addr, uint8_t *data_out, size_t len) {
  // Ensure enough space in cache
  if (col_addr + len > NAND_MAIN_PAGE_SIZE + NAND_SPARE_PAGE_SIZE_ECC_ON) {
    return FLASH_FAIL;
  }

  // load page from flash memory into cache
  // pg. 18 of datasheet
  cmd_spec page_load_cmd = {
    .opcode = CMD_NAND_PAGE_READ,
    .addr = row_addr,
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
    .addr = col_addr,
    .addr_size = QSPI_ADDRESS_16_BITS,
    .data_size = static_cast<uint32_t>(len),
    .dummy_cycles = 8,
    .data_lines = QSPI_DATA_4_LINES
  };
  
  cmd = build_cmd(cache_send_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // receive data from cache across spi
  QSPI_RECV_OR_RETURN(data_out, FLASH_FAIL);

  // Now check ECC is fine
  // load status register
  flash_status_a_t status_a{};

  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  // check whether read was ok
  // see pg. 46
  if (status_a.ECCS == UNCORRECTED_ERROR) {
    return FLASH_BAD_BLOCK;
  }

  return FLASH_SUCCESS;
}

bool operation_in_progress() {
  flash_status_a_t data;
  if (!get_status_a(&data)) {
    return false;
  }

  // fail if OIP is set
  if (data.OIP) {
    return false;
  }

  return true;
}

bool disable_flash_write() {
  cmd_spec write_disable_cmd = {
    .opcode = CMD_NAND_WRITE_DISABLE
  };

  QSPI_CommandTypeDef cmd = build_cmd(write_disable_cmd);
  if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  return true;
}

bool enable_flash_write() {
  cmd_spec write_enable_cmd = {
    .opcode = CMD_NAND_WRITE_DISABLE
  };

  QSPI_CommandTypeDef cmd = build_cmd(write_enable_cmd);
  if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TIMEOUT_MS) != HAL_OK) {
    disable_flash_write();
    return false;
  }

  return true;
}

flash_error_t send_cmd_or_fail(QSPI_CommandTypeDef *cmd) {
  if (HAL_QSPI_Command(&hqspi, cmd, QSPI_TIMEOUT_MS) != HAL_OK) {
    disable_flash_write();
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

} // namespace


bool begin() {
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_CLK), PinMap_QUADSPI_SCLK);
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_IO0), PinMap_QUADSPI_DATA0);
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_IO1), PinMap_QUADSPI_DATA1);
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_IO2), PinMap_QUADSPI_DATA2);
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_IO3), PinMap_QUADSPI_DATA3);
  pinmap_pinout(digitalPinToPinName(PIN_QSPI_CS), PinMap_QUADSPI_SSEL);

  RETURN_AND_LOG_IF_FAILED(init_qspi_peripheral, "QSPI Init Failed"); 
  RETURN_AND_LOG_IF_FAILED(reset_and_check_id, "NAND reset/ID check failed");
  RETURN_AND_LOG_IF_FAILED(disable_block_protection, "NAND disable block protection failed");
  RETURN_AND_LOG_IF_FAILED(enable_quad_mode, "NAND enable quad mode failed");
  RETURN_AND_LOG_IF_FAILED(ensure_ecc_enabled, "ECC is not enabled");

  CommsSerial.println("NAND flash driver ready");

  return true;
}

flash_error_t read(uint32_t row_addr, uint32_t col_addr, uint8_t *out, size_t len) {
  RETURN_IF_NULL(out, FLASH_FAIL);

  return flash_read_to_buffer(row_addr, col_addr, out, len);
}

flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len) {
  RETURN_IF_NULL(data, FLASH_FAIL);

  // does cache page have enough space?
  if (col_addr + len > NAND_MAIN_PAGE_SIZE + NAND_SPARE_PAGE_SIZE_ECC_ON) {
    return FLASH_FAIL;
  }

  // Program Load at col 0 clears the cache, Load Random keeps existing content
  uint32_t nand_cmd = col_addr == 0 ? CMD_NAND_PROGRAM_LOAD : CMD_NAND_PROGRAM_LOAD_RANDOM;

  // load data into cache
  cmd_spec load_cmd = {
    .opcode = nand_cmd, 
    .addr = col_addr, 
    .addr_size = QSPI_ADDRESS_16_BITS, 
    .data_size = len,
  };

  // Give cache metadata, so it knows how much data to expect
  QSPI_CommandTypeDef cmd = build_cmd(load_cmd);
  QSPI_CMD_OR_RETURN(&cmd, FLASH_FAIL);

  // send data across spi into cache
  QSPI_TRANSMIT_OR_RETURN(data, FLASH_FAIL);

  return FLASH_SUCCESS;
}

flash_error_t program(uint32_t row_addr) {
  if (!operation_in_progress()) {
    return FLASH_FAIL;
  }

  if (!enable_flash_write()) {
    return FLASH_FAIL;
  }

  // Now write in the page
  cmd_spec commit_cache_cmd = {
    .opcode = CMD_NAND_PROGRAM_EXECUTE, 
    .addr = row_addr, 
    .addr_size = QSPI_ADDRESS_24_BITS
  };

  QSPI_CommandTypeDef commit_cmd = build_cmd(commit_cache_cmd);
  send_cmd_or_fail(&commit_cmd);
  
  if (!disable_flash_write()) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

flash_error_t check_program_status() {
  if (!operation_in_progress()) {
    return FLASH_FAIL;
  }

  // ensure successful previous program
  flash_status_a_t status_a{};
  
  if (!get_status_a(&status_a)) {
    return FLASH_FAIL;
  }

  // Program failed operation, assume cause is bad block
  if (status_a.P_FAIL) {
    return FLASH_BAD_BLOCK;
  }

  // Previous program succeeded
  return FLASH_SUCCESS;
}

flash_error_t erase_block(uint32_t row_addr) {
  // ensure chip is done with any previous erases
  flash_error_t err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  if (err != FLASH_READY) {
    return err;
  }

  if (!enable_flash_write()) {
    return FLASH_FAIL;
  }

  // Erase block
  cmd_spec erase_cmd = {
    .opcode = CMD_NAND_BLOCK_ERASE, 
    .addr = row_addr, 
    .addr_size = QSPI_ADDRESS_24_BITS
  };
  
  QSPI_CommandTypeDef erase = build_cmd(erase_cmd);
  if (!send_cmd_or_fail(&erase)) {
    return FLASH_FAIL;
  }

  // wait for erase to finish
  err = wait_until_ready(WORST_CASE_MEMORY_TIMEOUT);
  if (err != FLASH_READY) {
    disable_flash_write();
    return err;
  }

  // ensure erase was successful
  flash_status_a_t status_a = {};

  // load status register
  if (!get_status_a(&status_a)) {
    disable_flash_write();
    return FLASH_FAIL;
  }

  // mark block as bad if erase failed
  if (status_a.E_FAIL) {
    disable_flash_write();
    return FLASH_BAD_BLOCK;
  }

  if (!disable_flash_write()) {
    return FLASH_FAIL;
  }

  return FLASH_SUCCESS;
}

}; // namespace Flash
