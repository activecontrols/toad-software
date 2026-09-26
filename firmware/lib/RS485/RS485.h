#pragma once
#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#if defined(USE_HALV2_DRIVER)
#error "RS485Bus is written against the legacy STM32 HAL UART handle (UART_HandleTypeDef)"
#endif

// RS485 bus on an STM32 U(S)ART with hardware driver-enable (DE).
//
// Init sequence (begin()):
//   1. DE held LOW as a GPIO (transceiver in receive), all selects LOW.
//   2. Uart::begin(): core enables clocks, muxes RX/TX, sets up NVIC,
//      runs HAL_UART_Init and arms interrupt-driven RX.
//   3. HAL_RS485Ex_Init() on the core's own handle: DE mode, polarity, DEAT/DEDT.
//   4. RX re-armed through the core (step 3 resets RxState, orphaning the
//      core's pending Receive_IT).
//   5. DE pin muxed to its USART AF with the core's pinmap_pinout(), only after
//      the peripheral is already in DE mode, so the pin is never a live RTS output.
//
// begin()/end() are virtual in Uart, so every (re)init path, including a plain
// bus.begin(baud), goes through the RS485 configuration.
class RS485Bus : public Uart {
public:
  // N is deduced from the select-pin array. DE is deliberately NOT passed to the
  // core as RTS: that would enable RTS flow control and make the pin a live RTS
  // output during init. See muxDE().
  template <size_t N>
  RS485Bus(uint32_t rx, uint32_t tx, uint32_t de, const uint32_t (&sels)[N])
      : Uart(rx, tx), de_(de), sels_(sels), sel_count_(N) {}

  // Only pointer to sels is stored, so a temporary array would dangle:
  // RS485Bus b(rx, tx, de, {PIN_A, PIN_B}) must not compile.
  template <size_t N> RS485Bus(uint32_t rx, uint32_t tx, uint32_t de, const uint32_t (&&sels)[N]) = delete;

  using Uart::begin;                                        // keep begin(baud) visible alongside the override below
  void begin(unsigned long baud, uint16_t config) override; // config must be SERIAL_8N1
  void end() override;
  operator bool() override; // core init OK and DE mode confirmed in hardware

  bool setBaud(uint32_t baud); // in-place reconfigure; waits for TX to drain first
  uint32_t baud() const {
    return baud_;
  }

  bool waitTxComplete(uint32_t timeout_us);
  uint32_t frameTimeUs(size_t len) const; // 8N1: 10 bits per byte
  void deselectAll();

private:
  friend class RS485Device;
  void select(size_t idx);
  bool configureRS485(uint32_t baud); // HAL_RS485Ex_Init on the core handle + RX re-arm
  bool muxDE();                       // DE pin -> USART AF via the core's pinmap

  const uint32_t de_;
  const uint32_t *const sels_;
  const size_t sel_count_;
  uint32_t baud_ = 0;
  bool rs485_ok_ = false;
};

class RS485Device {
public:
  RS485Device(RS485Bus &bus, size_t idx, uint32_t baud) : bus(bus), idx_(idx), baud_(baud) {}

  // Deselects all, switches the bus to this device's baud if needed, selects this
  // device and flushes stale RX. Returns false (with nothing selected) on failure.
  bool beginTransaction();
  void endTransaction();

  // Returns number of bytes received (== len on success).
  size_t read(uint8_t *dst, size_t len, uint32_t latency_us = 500);

  void setBaud(uint32_t baud) {
    baud_ = baud;
  } // applied at next beginTransaction()
  uint32_t baud() const {
    return baud_;
  }

  RS485Bus &bus; // is-a Uart: use bus.write() etc. inside a transaction

private:
  const size_t idx_;
  uint32_t baud_;
};

namespace RS485s {
bool begin();

extern RS485Bus bus6;
extern RS485Bus bus2;

extern RS485Device tvc_pitch;
extern RS485Device enc_ox;
extern RS485Device drv_ox;
extern RS485Device tvc_yaw;
extern RS485Device enc_fu;
extern RS485Device drv_fu;
} // namespace RS485s