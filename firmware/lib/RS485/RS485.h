#pragma once
#include <Arduino.h>
#include <cstddef>
#include <cstdint>


// DE pin is passed to the Uart constructor as RTS so that the core muxes it,
// and RS485Bus converts RTS flow control into DE mode.
// NOTE: DO NOT call uart.begin() on a bus after RS485s::begin() since it will re-enable RTSE and kill DE.
class RS485Bus {
public:
  RS485Bus(Uart &uart, const uint32_t *sels, size_t sel_count);

  bool begin(uint32_t baud);   // once, at startup
  bool setBaud(uint32_t baud); // waits for TX to finish 
  uint32_t baud() const { return baud_; }

  void deselectAll();
  bool waitTxComplete(uint32_t timeout_us);
  uint32_t frameTimeUs(size_t len) const; // 8N1: 10 bits per byte
  bool deModeActive() const;

private:
  Uart &uart_;
  const uint32_t *sels_;
  size_t sel_count_;
  uint32_t baud_ = 0;

  void select(size_t idx);
  static void applyDE(USART_TypeDef *u); // requires UE = 0

  friend class RS485Device;
};

class RS485Device {
public:
  RS485Device(RS485Bus &bus, size_t idx, uint32_t baud)
      : uart(bus.uart_), bus_(bus), idx_(idx), baud_(baud) {}

  // Switches bus to this device's baud if needed, selects it, flushes stale RX.
  // Returns false if baud switch failed.
  bool beginTransaction();
  void endTransaction();

  // Returns number of bytes received (== len on success).
  size_t read(uint8_t *dst, size_t len, uint32_t latency_us = 500);

  void setBaud(uint32_t baud) { baud_ = baud; } // applied at next beginTransaction()
  uint32_t baud() const { return baud_; }

  Uart &uart; 

private:
  RS485Bus &bus_;
  size_t idx_;
  uint32_t baud_;
};

namespace RS485s {
bool begin();

extern RS485Bus bus6;
extern RS485Bus bus2;

extern RS485Device tvc_pitch;
extern RS485Device enc_ox;
extern RS485Device tvc_yaw;
extern RS485Device enc_fu;
} // namespace RS485s