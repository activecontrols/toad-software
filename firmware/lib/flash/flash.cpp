#include "flash.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "fc_pins.h"
#include "flash_defs.h"
#include <Arduino.h>
#include <stm32h747xx.h>

#define FREQ_MAX 50'000'000

#define HAL_TIMEOUT 10000

// proper practice here is to always keep WEL disabled (write enable latch, which should be kept low to prevent accidental writes or erases)
// each function that modifies data on the flash should first enable WEL, write, then disable WEL before returning

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi) {
  __HAL_RCC_QSPI_CLK_ENABLE();

  __HAL_RCC_QSPI_FORCE_RESET();

  __HAL_RCC_QSPI_RELEASE_RESET();

  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init;

  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_10;
  gpio_init.Alternate = GPIO_AF9_QUADSPI;

  HAL_GPIO_Init(GPIOF, &gpio_init);

  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  gpio_init.Alternate = GPIO_AF9_QUADSPI; // or AF10 depending on part
  HAL_GPIO_Init(GPIOD, &gpio_init);

  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Alternate = GPIO_AF10_QUADSPI;

  HAL_GPIO_Init(GPIOG, &gpio_init);
}

namespace Flash {
QSPI_HandleTypeDef hqspi;

bool command(uint8_t instruction, unsigned int data_len = 0, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = instruction;
  cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
  cmd.DummyCycles = dummy;
  cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (data_len) {
    cmd.NbData = data_len;
    cmd.DataMode = QSPI_DATA_4_LINES;
  }

  if (addr != static_cast<uint32_t>(-1)) {
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_4_LINES;
  }

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) != HAL_OK)
    return false;

  return true;
}

bool receive(uint8_t instruction, void *data, unsigned int len, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  if (!command(instruction, len, addr, dummy))
    return false;

  return HAL_QSPI_Receive(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

// note that this function WILL NOT wait for the write operation to complete; you need to check WIP bit separately
bool write(uint8_t instruction, void *data, unsigned int len, uint32_t addr = static_cast<uint32_t>(-1), unsigned int dummy = 0) {
  if (!command(instruction, len, addr, dummy))
    return false;

  return HAL_QSPI_Transmit(&hqspi, static_cast<uint8_t *>(data), HAL_TIMEOUT) == HAL_OK;
}

// returns true if peripheral did not error
bool enable_qspi() {
  QSPI_CommandTypeDef cmd{};

  cmd.Instruction = CMD_EQIO;                        // enable QSPI command - EQIO
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;     // InstructionMode 1 corresponds to single serial mode - use 1 pin for output and 1 pin for input
  cmd.AddressMode = QSPI_ADDRESS_NONE;               // skip sending address
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE; // skip alternate bytes
  cmd.DummyCycles = 0;
  cmd.DataMode = QSPI_DATA_NONE;
  cmd.NbData = 0;
  cmd.DdrMode = 0;

  // when sending a command without data the command initiates when HAL_QSPI_Command is called
  return HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT) == HAL_OK;
}

// read electronic id over quadspi
// returns true if peripheral did not error
inline bool read_qpiid(uint8_t out[3]) {
  return receive(CMD_QPIID, out, 3, -1, 6);
}
// read status register (RDSR)
inline bool read_status(flash_status_reg *out) {
  return receive(CMD_RDSR, out, sizeof(*out));
}

// read security register (RDSCUR)

// wait for write-in-progress bit to be unset
bool wait_for_wip(unsigned long max_delay) {
  flash_status_reg reg;

  unsigned long start_micros = micros();
  do {
    unsigned long now_micros = micros();

    if (now_micros - start_micros > max_delay)
      return false;

    read_status(&reg);
    delayMicroseconds(10);
  } while (reg.WIP);

  return true;
}

bool is_write_enable() {
  flash_status_reg reg;

  if (!read_status(&reg))
    return false;

  return reg.WEL;
}

bool is_write_in_progress() {
  flash_status_reg reg;

  if (!read_status(&reg))
    return true;

  return reg.WIP;
}

// note: this function does not wait for WIP to be set before returning
inline bool write_enable() {
  return command(CMD_WREN);
}

// note: this function does not wait for WIP to be unset before returning
inline bool write_disable() {
  return command(CMD_WRDI);
}

// !!! EXERCISE CAUTION: THIS COMMAND WILL ERASE THE ENTIRE CHIP !!!
// this function blocks until the erase is complete
bool chip_erase() {
  // max chip erase time per DS is 60 seconds
  const unsigned int timeout_us = 70'000'000;
  wait_for_wip();
  write_enable();
  wait_for_wip();

  // check that write enable bit is set
  if (!is_write_enable())
    return false;

  if (!command(CMD_CE))
    return false;

  if (!write_disable())
    return false;
  if (!wait_for_wip(timeout_us))
    return false; // wait for chip erase operation to complete (can take up to 60 seconds)

  if (is_write_enable())
    return false;

  return true;
}

// erase a 4KB sector
bool sector_erase(uint32_t addr) {
  const unsigned int timeout_us = 240'000; // DS says max sector erase time is 120ms
  wait_for_wip();
  write_enable();
  wait_for_wip();

  // check that write enable bit is set
  if (!is_write_enable()) {
    return false;
  }

  // send sector erase command
  if (!command(CMD_SE, 0, addr))
    return false;

  write_disable(); // disable write to protect against accidental data corruption

  if (!wait_for_wip(timeout_us)) // wait for erase operation to complete
    return false;

  // check the write enable bit is unset
  if (is_write_enable())
    return false;

  return true;
}

// as per DS - only last 256 bytes are actually written, and the write address wraps back around the the beginning of the 256-byte page when necessary
// chain function calls together to write data across multiple pages
bool page_program(uint32_t addr, uint8_t *data, uint32_t len) {
  if (!wait_for_wip(1500) || !write_enable() || !wait_for_wip())
    return false;

  bool status = write(CMD_PP, data, len, addr);
  write_disable();
  return status;
}

bool _initialize() {
  // reset the flash device
  digitalWrite(PIN_QSPI_FLASH_RST_IO3, HIGH);
  pinMode(PIN_QSPI_FLASH_RST_IO3, OUTPUT);
  digitalWrite(PIN_QSPI_FLASH_RST_IO3, LOW);
  delay(350);
  digitalWrite(PIN_QSPI_FLASH_RST_IO3, HIGH);

  uint32_t f_hclk = HAL_RCC_GetHCLKFreq();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = f_hclk / FREQ_MAX;

  if (hqspi.Init.ClockPrescaler > 255) {
    hqspi.Init.ClockPrescaler = 255;
  } else if (!(f_hclk % FREQ_MAX)) // "round up" when needed
  {
    --hqspi.Init.ClockPrescaler;
  }

  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;

  HAL_StatusTypeDef status = HAL_QSPI_Init(&hqspi);

  if (status != HAL_OK) {
    CommsSerial.print("HAL_QSPI_Init failed.\n");
    return false;
  }

  CommsSerial.print("QSPI Peripheral Initialized.\n");
  CommsSerial.printf("HCLK Frequency: %d\n", f_hclk);
  CommsSerial.printf("QSPI Prescaler: %d\n", hqspi.Init.ClockPrescaler);
  CommsSerial.print("QSPI Frequency: ");
  CommsSerial.print(f_hclk / (hqspi.Init.ClockPrescaler + 1) * 1e-6, 2);
  CommsSerial.print(" MHz\n");

  if (!enable_qspi()) {
    CommsSerial.print("Flash enable QPIO command (EQIO) failed.\n");
    return false;
  }

  HAL_QSPI_StateTypeDef state = HAL_QSPI_GetState(&hqspi);

  CommsSerial.print("QSPI Peripheral State: ");
  CommsSerial.print(state, HEX);
  CommsSerial.print('\n');

  uint8_t qpiid[3];
  const uint8_t correct_qpiid[3] = {0xC2, 0x20, 0x18};
  if (!read_qpiid(qpiid)) {
    CommsSerial.print("Read QPIID failed.\n");
    return false;
  }

  CommsSerial.print("QPIID: ");
  for (int i = 0; i < sizeof(qpiid); ++i) {
    CommsSerial.print(qpiid[i], HEX);
    CommsSerial.print(' ');
  }
  CommsSerial.print('\n');

  if (memcmp(qpiid, correct_qpiid, sizeof(correct_qpiid)) != 0) {
    CommsSerial.print("QPIID response invalid.\n");
    return false;
  }

  return true;
}

bool read(uint32_t addr, uint32_t len, uint8_t *out) {
  wait_for_wip();
  return receive(CMD_4READ, out, len, addr, 6);
}

void rw_test() {
  const uint32_t addr = 0x0FF000;
  const uint32_t sector = SECTOR(addr);
  const uint32_t page = PAGE(addr);

  CommsSerial.printf("Erasing Sector %d\n", sector);
  if (!sector_erase(addr)) {
    CommsSerial.print("Sector erase command failed.\n");
    CommsSerial.printf("HAL Error: %X\n", HAL_QSPI_GetError(&hqspi));
    return;
  }
  CommsSerial.print("Sector erased and WIP unset.\n");

  // program sector
  char program_data[256];
  char str[] = "Hello, world! From the NOR Flash!";
  memset(program_data, 0, sizeof(program_data));
  memcpy(program_data, str, sizeof(str));

  CommsSerial.printf("Programming page %d\n", page);
  unsigned long start_micros = micros();

  if (!page_program(addr, reinterpret_cast<uint8_t *>(program_data), sizeof(program_data))) {
    CommsSerial.print("Page program command Failed.\n");
    return;
  }

  // wait for write operation to finish
  // DS says this could take up to 1.2 ms
  if (!wait_for_wip(1500)) {
    CommsSerial.print("wait_for_wip failed\n");
    return;
  }
  unsigned long delta_t = micros() - start_micros;
  CommsSerial.print("Page program command succeeded.\n");
  CommsSerial.print("WIP Unset.\n");
  CommsSerial.printf("Single page write time (us): %d\n", delta_t);

  CommsSerial.printf("Reading page %d\n", page);

  char data_read[sizeof(program_data)];
  if (!read(addr, sizeof(data_read), reinterpret_cast<uint8_t *>(data_read))) {
    CommsSerial.print("4READ command failed\n");
    return;
  }

  if (memcmp(data_read, program_data, sizeof(program_data)) != 0) {
    CommsSerial.print("Error: 4READ data does not match program data.\n");
  }

  CommsSerial.printf("Page %d contents: ", page);
  CommsSerial.print(data_read);
  CommsSerial.print('\n');
}

void write_speed_test() {
  const unsigned long addr = 0xFFFF00; // use the top page/sector

  uint8_t program_data[256];
  memset(program_data, 'A', sizeof(program_data));

  for (unsigned long i = 5; i < sizeof(program_data); i += 10) {
    if (!sector_erase(addr)) {
      CommsSerial.print("Sector Erase Failed\n");
      return;
    }

    unsigned long start_micros = micros();
    if (!page_program(addr, program_data, i)) {
      CommsSerial.print("Page Program Failed\n");
      return;
    }

    if (!wait_for_wip(2'400)) {
      CommsSerial.print("wait_for_wip failed.\n");
      return;
    }

    unsigned long delta_micros = micros() - start_micros;

    if (is_write_enable()) {
      CommsSerial.print("Error: write is still enabled\n");
      return;
    }

    CommsSerial.print(i);
    CommsSerial.print(", ");
    CommsSerial.print(delta_micros);
    CommsSerial.print('\n');

    delay(10);
  }
}

void begin() {

  if (!_initialize()) {
    while (1) {
      CommsSerial.print("QSPI Init Failed\n");
      delay(1000);
    }
  }

  CommsSerial.print("Flash driver ready.\n");
  CommandRouter::add(rw_test, "rw_test");
  CommandRouter::add(write_speed_test, "write_speed_test");

  return;
}
}; // namespace Flash