#include "RS485.h"
#include "CommsSerial.h"
#include "ec_pins.h"
#include <iterator>

namespace {
constexpr uint32_t kMarginUs = 200; // TODO: tune against response time
constexpr uint32_t kTxSlackUs = 2000;
} // namespace

RS485Bus::RS485Bus(Uart &uart, const uint32_t *sels, size_t sel_count)
    : uart_(uart), sels_(sels), sel_count_(sel_count) {}

bool RS485Bus::begin(uint32_t baud) {
  for (size_t i = 0; i < sel_count_; i++) {
    pinMode(sels_[i], OUTPUT);
    digitalWrite(sels_[i], LOW);
  }

  uart_.begin(baud); // core muxes RX/TX/DE (as RTS) and sets RTSE
  if (!uart_)
    return false;
  return setBaud(baud); // also converts RTS flow control -> DE mode
}

bool RS485Bus::setBaud(uint32_t baud) {
  if (baud == 0)
    return false;
  if (!waitTxComplete(frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs))
    return false;

  UART_HandleTypeDef *h = uart_.getHandle();
  USART_TypeDef *u = h->Instance;

  h->Init.BaudRate = baud;
  h->Init.HwFlowCtl = UART_HWCONTROL_NONE; // stop UART_SetConfig from re-enabling RTSE

  noInterrupts();
  // UART_SetConfig clears the HAL ISR pointers the core's IT-mode RX depends on.
  auto rx_isr = h->RxISR;
  auto tx_isr = h->TxISR;

  u->CR1 &= ~USART_CR1_UE;                 // BRR/DE fields writable only with UE = 0
  bool ok = (UART_SetConfig(h) == HAL_OK); // BRR + PRESC from the real clock source
  applyDE(u);
  u->CR1 |= USART_CR1_UE;

  h->RxISR = rx_isr;
  h->TxISR = tx_isr;
  interrupts();

  while (uart_.available())
    uart_.read(); // anything framed at the old rate is garbage
  if (ok)
    baud_ = baud;
  return ok;
}

void RS485Bus::applyDE(USART_TypeDef *u) {
  u->CR3 &= ~(USART_CR3_RTSE | USART_CR3_DEP); // no RTS flow control, DE active-high
  u->CR3 |= USART_CR3_DEM;
  u->CR1 &= ~(USART_CR1_DEAT | USART_CR1_DEDT);

  // DEAT = 31/16 bit: gives the transceiver time to enable
  u->CR1 |= (31U << USART_CR1_DEAT_Pos) | (1U << USART_CR1_DEDT_Pos);
}

bool RS485Bus::deModeActive() const {
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

// Same condition Uart::flush() waits on (tx_tail advances in the TC callback), but bounded.
bool RS485Bus::waitTxComplete(uint32_t timeout_us) {
  const uint32_t start = micros();
  while (uart_.availableForWrite() < SERIAL_TX_BUFFER_SIZE - 1) {
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

  const uint32_t budget = latency_us + bus_.frameTimeUs(len) + kMarginUs;
  const uint32_t start = micros();
  size_t n = 0;
  if (complete) {
    while (n < len) {
      if (uart.available()) {
        dst[n++] = (uint8_t)uart.read();
        continue;
      }
      if (micros() - start >= budget)
        break;
    }
    return n;
  }
  return 0;
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

RS485Bus bus6(RS485_6, bus6_sels, std::size(bus6_sels));
RS485Bus bus2(RS485_2, bus2_sels, std::size(bus2_sels));

RS485Device tvc_pitch(bus6, 0, kTvcBaud);
RS485Device enc_ox(bus6, 1, kEncBaud);
RS485Device drv_ox(bus6, 2, kDrvBaud);

RS485Device tvc_yaw(bus2, 0, kTvcBaud);
RS485Device enc_fu(bus2, 1, kEncBaud);
RS485Device drv_fu(bus2, 2, kDrvBaud);

bool begin() {
  bool ok = true;
  ok &= bus6.begin(kEncBaud); // start at the in-flight rate
  ok &= bus2.begin(kEncBaud);
  ok &= bus6.deModeActive();
  ok &= bus2.deModeActive();
  if (!ok)
    CommsSerial.println("ERROR: RS485 init failed");
  return ok;
}
} // namespace RS485s