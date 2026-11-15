#pragma once
#include "Arduino.h"
#include "CommandRouter.h"
#include "api/HardwareCAN.h"
#include <stdint.h>

#if defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32G4)
#include "stm32g474xx.h"
#else
#error Error
#endif

class CAN : public arduino::HardwareCAN {
public:
  CAN(uint32_t tx_pin, uint32_t rx_pin);

  enum class error_type_t { TX_ERROR, RX_ERROR, RX_ERROR_PASSIVE };

  // Initialize using standard Arduino CanBitRate enum.
  // Conforms to arduino::HardwareCAN interface.
  // Returns true on success, false on failure.
  bool begin(CanBitRate bit_rate) override;

  // Settings for starting a bus.
  struct config_t {
    CanBitRate nominal_bit_rate = CanBitRate::BR_500k; // arbitration phase (and the whole frame without BRS)
    uint32_t mode = FDCAN_MODE_NORMAL; // or FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_EXTERNAL_LOOPBACK for testing
    uint32_t frame_format = FDCAN_FRAME_CLASSIC; // or FDCAN_FRAME_FD_NO_BRS / FDCAN_FRAME_FD_BRS
    uint32_t data_bit_rate = 8'000'000;          // data phase in bit/s, only used with FDCAN_FRAME_FD_BRS
  };

  // CAN-FD message type
  struct fd_msg_t {
    uint32_t id = 0;
    uint8_t len = 0; // number of data bytes, 0-64
    uint8_t data[64] = {0};
  };

  // Transmit a CAN-FD frame. The bus must have been started with an FD frame_format.
  // On an FDCAN_FRAME_FD_BRS bus the data phase is sent at data_bit_rate.
  // Lengths that are not a valid FD size are padded with zeros to the next valid size (e.g. 10 -> 12).
  // Returns 1 on success, 0 on error or timeout.
  int write_fd(const fd_msg_t &msg);

  // Take the next received frame (classic or FD) out of the receive FIFO.
  // Returns true and fills msg if a frame was waiting, false if not.
  bool read_fd(fd_msg_t &msg);

  // Initialize with full control over the settings.
  // Returns true on success, false on failure.
  bool begin(const config_t &config);

  // Sends a known frame and checks that it comes back unchanged.
  // Only passes when the bus was started in FDCAN_MODE_INTERNAL_LOOPBACK or FDCAN_MODE_EXTERNAL_LOOPBACK.
  static void loopback_test(CAN &bus);

  // Sends FD frames of 8, 10, 20 and 64 bytes and checks each comes back with the right length,
  // data and zero padding. Needs an FD frame_format and a loopback mode.
  static void loopback_test_fd(CAN &bus);

  // Get number of messages waiting in the receive FIFO.
  // Conforms to arduino::HardwareCAN interface.
  size_t available(void) override;

  // Get number of free positions in the transmit buffer (1 if free, 0 if busy).
  uint32_t tx_free_count(void);

  // Transmit a data frame in classic CAN mode with an 11-bit or 29-bit ID.
  // Conforms to arduino::HardwareCAN interface.
  // Returns 1 on success, 0 on error or timeout.
  int write(arduino::CanMsg const &msg) override;

  // Retrieve the next received message from the receive FIFO.
  // Conforms to arduino::HardwareCAN interface.
  // Returns an empty CanMsg if no message was available or an error occurred.
  arduino::CanMsg read(void) override;

  // Disable the FDCAN peripheral.
  // Conforms to arduino::HardwareCAN interface.
  void end(void) override;

  // Wait for ongoing transmission in TX buffer 0 to complete.
  // Returns true if buffer is free, false if timed out.
  bool flush(uint32_t timeout_ms = UINT32_MAX);

  /* Test sending data on one CAN, and expect to receive data after every message */
  void cmd_can_send_test(void);

  /* Test receiving data on one CAN, and send a frame after every received message */
  void cmd_can_receive_test(void);

private:
  FDCAN_HandleTypeDef hfdcan = {0};
  uint32_t tx_pin = 0;
  uint32_t rx_pin = 0;
  uint32_t frame_format = FDCAN_FRAME_CLASSIC;
};

template <CAN &sender, CAN &receiver> void cmd_can_loopback(void) {
  // repeat until the user presses a button
  while (!CommsSerial.available()) {
    unsigned long count = 0;
    CommsSerial.printf("Test %u:\n", count);
    CanMsg test_msg(0x123, 8, (const uint8_t *)"HELLO123");

    sender.write(test_msg);

    uint32_t start_ms = millis();
    while (!receiver.available()) {
      if (millis() - start_ms > 500) {
        CommsSerial.println("Receive timed out");
        return;
      }
    }

    while (receiver.available()) {
      CanMsg received = receiver.read();

      CommsSerial.printf("Received message: ID: 0x%04X\tDLC: %2d\tData: %.*s\n", received.id, received.data_length,
                         received.data_length, (const char *)received.data);
    }

    delay(500);
    ++count;
  }
}