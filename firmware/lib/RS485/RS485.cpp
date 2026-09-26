#include "RS485.h"
#include "CommsSerial.h"
#include "PeripheralPins.h"
#include "ec_pins.h"
#include "pinmap.h"
#include "stm32yyxx_ll_usart.h"

namespace {
constexpr uint32_t kMarginUs = 200; // TODO: tune against response time
constexpr uint32_t kTxSlackUs = 2000;

// DE timing, in sample-time units (1/16 bit at OVER16); HAL range 0..31.
// Assertion: DE rises this long before the start bit. Must exceed the transceiver's
// assert/deassert time is (parameter) / (baud * oversampling_ratio)
// for 16x oversampling: (parameter) / (baud * 16)
constexpr uint32_t kDeAssertTime = 16;
// Deassertion: DE held this long after the end of the last stop bit.
constexpr uint32_t kDeDeassertTime = 16;
} // namespace

void RS485Bus::begin(unsigned long baud, uint16_t config) {
  rs485_ok_ = false;

  // Transceiver in receive and all devices deselected until the USART owns DE.
  pinMode(de_, OUTPUT);
  digitalWrite(de_, LOW);
  for (size_t i = 0; i < sel_count_; i++) {
    pinMode(sels_[i], OUTPUT);
    digitalWrite(sels_[i], LOW);
  }

  Uart::begin(baud, config);                          // clocks, RX/TX mux, NVIC, HAL_UART_Init, RX armed
  if (!Uart::operator bool() || config != SERIAL_8N1) // frameTimeUs() assumes 8N1
    return;

  rs485_ok_ = configureRS485(baud) && muxDE();
}

void RS485Bus::end() {
  rs485_ok_ = false;
  Uart::end();
  // Take DE back from the now-unclocked USART and hold receive.
  pinMode(de_, OUTPUT);
  digitalWrite(de_, LOW);
}

RS485Bus::operator bool() {
  // rs485_ok_ first: before begin() the handle's Instance is null.
  return rs485_ok_ && Uart::operator bool() && LL_USART_IsEnabledDEMode(getHandle()->Instance);
}

// Reconfigure the core's own handle for RS485, in place.
// HAL_RS485Ex_Init re-runs UART_SetConfig and UART_CheckIdleState; the latter sets
// RxState = READY, which orphans the core's pending 1-byte Receive_IT (the ISR would
// then discard every byte). So: stop RX explicitly, reconfigure, re-arm RX through
// the core's own entry point, and verify it is armed.
//
// NOTE: UART_CheckIdleState waits for TEACK with HAL_UART_TIMEOUT_VALUE (~9 h), not a
// short timeout. It only blocks if the USART kernel clock is dead; the core's boot-time
// HAL_UART_Init has the same exposure.
bool RS485Bus::configureRS485(uint32_t baud) {
  UART_HandleTypeDef *h = getHandle();

  HAL_NVIC_DisableIRQ(_serial.irq); // core does the same around Receive_IT (handle lock)
  HAL_UART_AbortReceive(h);
  h->Init.BaudRate = baud;
  h->Init.HwFlowCtl = UART_HWCONTROL_NONE; // the pin is DE, not RTS
  const bool ok = HAL_RS485Ex_Init(h, UART_DE_POLARITY_HIGH, kDeAssertTime, kDeDeassertTime) == HAL_OK;
  HAL_NVIC_EnableIRQ(_serial.irq);
  if (!ok)
    return false;

  uart_attach_rx_callback(&_serial, _rx_complete_irq);
  if ((HAL_UART_GetState(h) & HAL_UART_STATE_BUSY_RX) != HAL_UART_STATE_BUSY_RX)
    return false; // RX not re-armed: fail closed

  baud_ = baud;
  return true;
}

// Mux DE with the same call the core uses for RX/TX/RTS. RTS and DE share one AF on
// STM32, so the RTS pinmap is the DE pinmap. The peripheral is checked first because
// pinmap_pinout() hangs in Error_Handler() on an unmapped pin, and because lookup is
// first-match: if this pin's DE function is on an _ALTx entry, fail loudly here rather
// than mux the wrong AF.
bool RS485Bus::muxDE() {
  const PinName pn = digitalPinToPinName(de_);
  if (pinmap_peripheral(pn, PinMap_UART_RTS) != (void *)getHandle()->Instance)
    return false;
  pinmap_pinout(pn, PinMap_UART_RTS);
  return true;
}

bool RS485Bus::setBaud(uint32_t baud) {
  if (!rs485_ok_ || baud == 0)
    return false;
  if (baud == baud_)
    return true;
  if (!waitTxComplete(frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs))
    return false;
  rs485_ok_ = configureRS485(baud); // no clock/pin/NVIC teardown, unlike end()+begin()
  return rs485_ok_;
}

// Done = core ring buffer drained, no HAL transfer in flight, and the last stop bit
// has left the shift register. TC alone can read 1 for a few cycles at a ring-buffer
// wrap, before the TX-complete ISR queues the next chunk.
bool RS485Bus::waitTxComplete(uint32_t timeout_us) {
  UART_HandleTypeDef *h = getHandle();
  const uint32_t start = micros();

  while (_serial.tx_head != _serial.tx_tail || serial_tx_active(&_serial) || !__HAL_UART_GET_FLAG(h, UART_FLAG_TC)) {
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

// RS485Device methods

bool RS485Device::beginTransaction() {
  bus.deselectAll();
  if (!bus.setBaud(baud_))
    return false; // never select a device at the wrong baud

  bus.select(idx_);
  while (bus.available())
    bus.read();
  return true;
}

size_t RS485Device::read(uint8_t *dst, size_t len, uint32_t latency_us) {
  // Don't start the response clock while our own request is still on the wire.
  if (!bus.waitTxComplete(bus.frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs)) {
    return 0; // bus still transmitting
  }

  const uint32_t budget = latency_us + bus.frameTimeUs(len) + kMarginUs;
  const uint32_t start = micros();
  size_t n = 0;

  while (n < len) {
    if (bus.available()) {
      dst[n++] = (uint8_t)bus.read();
      continue; // drain available bytes before checking the timeout
    }
    if (micros() - start >= budget) {
      break;
    }
  }
  return n;
}

void RS485Device::endTransaction() {
  bus.waitTxComplete(bus.frameTimeUs(SERIAL_TX_BUFFER_SIZE) + kTxSlackUs);
  bus.deselectAll();
}

// Namespace globals

namespace RS485s {
namespace {
constexpr uint32_t kEncBaud = 2'000'000; // AMT24 2 Mbps data rate
constexpr uint32_t kTvcBaud = 2'000'000; // TODO - check baud rate for TVC
constexpr uint32_t kDrvBaud = 115200;  // TODO - check driver's RS485 config; placeholder

// Index in each array == device index below
const uint32_t bus6_sels[] = {PIN_TVC_PITCH_SEL, PIN_ENC_OX_SEL, PIN_DRV_OX_SEL};
const uint32_t bus2_sels[] = {PIN_TVC_YAW_SEL, PIN_ENC_FU_SEL, PIN_DRV_FU_SEL};
} // namespace

RS485Bus bus6(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE, bus6_sels);
RS485Bus bus2(PIN_RS485_2_RX, PIN_RS485_2_TX, PIN_RS485_2_DE, bus2_sels);

RS485Device tvc_pitch(bus6, 0, kTvcBaud);
RS485Device enc_ox(bus6, 1, kEncBaud);
RS485Device drv_ox(bus6, 2, kDrvBaud);

RS485Device tvc_yaw(bus2, 0, kTvcBaud);
RS485Device enc_fu(bus2, 1, kEncBaud);
RS485Device drv_fu(bus2, 2, kDrvBaud);

bool begin() {
  bus6.begin(kEncBaud);
  bus2.begin(kEncBaud);
  const bool ok = static_cast<bool>(bus6) && static_cast<bool>(bus2);
  if (!ok)
    CommsSerial.println("ERROR: RS485 init failed");
  return ok;
}
} // namespace RS485s