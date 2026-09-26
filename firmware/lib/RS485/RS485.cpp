#include "RS485.h"
#include "CommsSerial.h"
#include "ec_pins.h"
#include <iterator> // used to calculate size of the array hosuing the select pins in RS485s namespace.
// TO-DO: Restructure to use std::array and remove sel_count parameter

namespace {
constexpr uint32_t kMarginUs = 200; // TODO: tune against response time
constexpr uint32_t kTxSlackUs = 2000;
constexpr uint32_t kAckTimeoutUs = 1000; // TEACK/REACK after UE re-enable

} // namespace

RS485Bus::RS485Bus(uint32_t rx, uint32_t tx, uint32_t de, const uint32_t *sels, size_t sel_count)
    : uart_(rx, tx, de), sels_(sels), sel_count_(sel_count) {}

bool RS485Bus::begin(uint32_t baud) {
  for (size_t i = 0; i < sel_count_; i++) {
    pinMode(sels_[i], OUTPUT);
    digitalWrite(sels_[i], LOW);
  }
  return initAt(baud);
}

bool RS485Bus::setBaud(uint32_t baud) {
  if (baud == 0)
    return false;
  if (!waitTxComplete(frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs))
    return false;
  uart_.end();
  return initAt(baud);
}

// RS485 delta from HAL_RS485Ex_Init. SetConfig/AdvFeatureConfig
// are already done by uart_.begin(). UART_CheckIdleState skipped because it resets RxState and
// would disarm the core's Receive_IT; TX idle, bus deselected.

bool RS485Bus::initAt(uint32_t baud) {
  uart_.begin(baud);

  USART_TypeDef *u = uart_.getHandle()->Instance;
  u->CR1 &= ~USART_CR1_UE;
  applyDE(u);
  u->CR1 |= USART_CR1_UE;

  const uint32_t ack = USART_ISR_TEACK | USART_ISR_REACK;
  const uint32_t start = micros();
  while ((u->ISR & ack) != ack) {
    if (micros() - start >= kAckTimeoutUs)
      return false;
  }
  baud_ = baud;
  return true;
}

void RS485Bus::applyDE(USART_TypeDef *u) {
  u->CR3 &= ~(USART_CR3_RTSE | USART_CR3_DEP); // no RTS flow control, DE active-high // call pinmap_pinout()
  u->CR3 |= USART_CR3_DEM;
  u->CR1 &= ~(USART_CR1_DEAT | USART_CR1_DEDT);

  // DEAT = 31/16 bit: gives the transceiver time to enable
  u->CR1 |= (31U << USART_CR1_DEAT_Pos) | (1U << USART_CR1_DEDT_Pos);
}

bool RS485Bus::deModeActive() {
  return (uart_.getHandle()->Instance->CR3 & USART_CR3_DEM) != 0;
}

void RS485Bus::deselectAll() {
  for (size_t i = 0; i < sel_count_; i++) {
    digitalWrite(sels_[i], LOW);
  }
}

void RS485Bus::select(size_t idx) {
  deselectAll();
  if (idx < sel_count_) {
    digitalWrite(sels_[idx], HIGH);
  }
}

// Wait for the hardware TX complete flag, not just the software TX buffer depth.
// previously used availableForWrite(), but that only tells us the queue has room, not that the final byte
// has left the shift register and the bus is safe to deselect.
bool RS485Bus::waitTxComplete(uint32_t timeout_us) {
  USART_TypeDef *u = uart_.getHandle()->Instance;
  const uint32_t start = micros();

  while ((u->ISR & USART_ISR_TC) == 0) {
    if (micros() - start >= timeout_us)
      return false;
  }

  return true;
}

uint32_t RS485Bus::frameTimeUs(size_t len) const {
  if (baud_ == 0)
    return 0;
  return (uint32_t)((len * 10ULL * 1000000ULL) / baud_);
}

// RS485Device methods

bool RS485Device::beginTransaction() {
  bus_.deselectAll();
  bool ok = true;
  if (bus_.baud_ != baud_)
    ok = bus_.setBaud(baud_);

  bus_.select(idx_);

  while (uart.available())
    uart.read();

  return ok;
}

size_t RS485Device::read(uint8_t *dst, size_t len, uint32_t latency_us) {
  // Don't start response clock while our own request is still sending
  bool complete = bus_.waitTxComplete(bus_.frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs);

  if (!complete) {
    return 0; // Error Code 0; Bus is still transmitting
  }

  const uint32_t budget = latency_us + bus_.frameTimeUs(len) + kMarginUs;
  const uint32_t start = micros();
  size_t n = 0;

  while (n < len) {
    if (uart.available()) {
      dst[n++] = (uint8_t)uart.read();
      continue; // ensure all bytes are read before checking timeout
    }

    if (micros() - start >= budget) {
      break;
    }
  }

  return n;
}

void RS485Device::endTransaction() {
  bus_.waitTxComplete(bus_.frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs);
  bus_.deselectAll();
}

// Namespace Globals

namespace RS485s {
namespace {
constexpr uint32_t kEncBaud = 2000000; // AMT24 2 Mbps data rate
constexpr uint32_t kTvcBaud = 2000000; // TODO - Check Baud rate for TVC
constexpr uint32_t kDrvBaud = 115200;  // TODO - check driver's RS485 config; placeholder

// Index in each array == device index below
const uint32_t bus6_sels[] = {PIN_TVC_PITCH_SEL, PIN_ENC_OX_SEL, PIN_DRV_OX_SEL};
const uint32_t bus2_sels[] = {PIN_TVC_YAW_SEL, PIN_ENC_FU_SEL, PIN_DRV_FU_SEL};
} // namespace

RS485Bus bus6(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE, bus6_sels, std::size(bus6_sels));
RS485Bus bus2(PIN_RS485_2_RX, PIN_RS485_2_TX, PIN_RS485_2_DE, bus2_sels, std::size(bus2_sels));

RS485Device tvc_pitch(bus6, 0, kTvcBaud);
RS485Device enc_ox(bus6, 1, kEncBaud);
RS485Device drv_ox(bus6, 2, kDrvBaud);

RS485Device tvc_yaw(bus2, 0, kTvcBaud);
RS485Device enc_fu(bus2, 1, kEncBaud);
RS485Device drv_fu(bus2, 2, kDrvBaud);

bool begin() {
  bool ok = true;
  ok &= bus6.begin(kEncBaud);
  ok &= bus2.begin(kEncBaud);
  ok &= bus6.deModeActive();
  ok &= bus2.deModeActive();
  if (!ok)
    CommsSerial.println("ERROR: RS485 init failed");
  return ok;
}
} // namespace RS485s