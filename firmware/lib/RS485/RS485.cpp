#include "RS485.h"
#include "CommsSerial.h"
#include "PeripheralPins.h"
#include "ec_pins.h"
#include "pinmap.h"


namespace {
  // in Microseconds
constexpr uint32_t kMarginUs = 200; // TODO: tune against response time
constexpr uint32_t kTxSlackUs = 2000;

// DE timing in USART sample times (1/16 bit at OVER16), written straight into
// CR1.DEAT/DEDT. Register range 0..31. A sample time scales with 1/baud, so these
// give the shortest real time at the highest baud: size them for kEncBaud (2 Mbps,
// 1 sample = 31.25 ns); lower bauds only get more margin.
//   Assert:   31 samples = 969 ns at 2 Mbps. Must exceed transceiver t_ZH / t_ZL.
//   Deassert:  1 sample  =  31 ns at 2 Mbps.
// Using sample time since STM32 splits each bit into samples 
constexpr uint32_t kDeAssertTime = 5; // IN SAMPLES
constexpr uint32_t kDeDeassertTime = 5; // IN SAMPLES
static_assert(kDeAssertTime <= 31 && kDeDeassertTime <= 31, "DEAT/DEDT are 5-bit fields");
} // namespace

void RS485Bus::begin(unsigned long baud, uint16_t config) {

  // Transceiver in receive and all devices deselected until the USART owns DE.
  pinMode(de_, OUTPUT);
  digitalWrite(de_, LOW);
  for (size_t i = 0; i < sel_count_; i++) {
    pinMode(sels_[i], OUTPUT);
    digitalWrite(sels_[i], LOW);
  }

  rs485_ok_ =  setBaud(baud) && configureDE();
}

void RS485Bus::end() {
  rs485_ok_ = false;
  Uart::end();
  // Take DE back from the now-unclocked USART and hold receive.
  pinMode(de_, OUTPUT);
  digitalWrite(de_, LOW);
}

// Configure the UART for half-duplex RS485: disable the peripheral while reprogramming
// the DE/RTS timing and polarity bits, 
// then re-enable the peripheral

void RS485Bus::configureRS485() {
  UART_HandleTypeDef *h = getHandle();

  __HAL_UART_DISABLE(h);                              // Clear UE so that DEM/DEP/DEAT/DEDT/BRR are writable
  CLEAR_BIT(h->Instance->CR3, USART_CR3_RTSE | USART_CR3_CTSE); // Explicityly clear RTSE since RTS and DE share same AF. Allows
                                                      // for safer hardware flow control

  SET_BIT(h->Instance->CR3, USART_CR3_DEM); // Enable Driver Enable mode in the CR3 register by setting DEM bit
  MODIFY_REG(h->Instance->CR3, USART_CR3_DEP, UART_DE_POLARITY_HIGH); // Set driver polarity high

  // Set driver enable assertion and deassertion times
  const uint32_t de_times =
      (kDeAssertTime << UART_CR1_DEAT_ADDRESS_LSB_POS) | (kDeDeassertTime << UART_CR1_DEDT_ADDRESS_LSB_POS);

  MODIFY_REG(h->Instance->CR1, (USART_CR1_DEDT | USART_CR1_DEAT), de_times);

  __HAL_UART_ENABLE(h);
}

// Mux DE with the same call the core uses for RX/TX/RTS. RTS and DE share one AF on
// STM32, so the RTS pinmap is the DE pinmap. The peripheral is checked first because
// pinmap_pinout() hangs in Error_Handler() on an unmapped pin, and because lookup is
// first-match: if this pin's DE function is on an _ALTx entry, fail loudly here rather
// than mux the wrong AF.
bool RS485Bus::configureDE() {
  const PinName pn = digitalPinToPinName(de_);
  if (pinmap_peripheral(pn, PinMap_UART_RTS) != (void *)getHandle()->Instance)
    return false;
  pinmap_pinout(pn, PinMap_UART_RTS);
  return true;
}

bool RS485Bus::setBaud(uint32_t baud) {
  if (baud == 0)
    return false;
  if (baud == baud_)
    return true;

  Uart::begin(baud, SERIAL_8N1);
  configureRS485();
  baud_ = baud;
  return Uart::operator bool(); // Checking underlying Uart ok
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
  if (!bus.ready() || !bus.setBaud(baud_))
    return false; // never select a device on a dead bus or at the wrong baud

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
constexpr uint32_t kEncBaud = 2000000; // AMT24 2 Mbps data rate
constexpr uint32_t kTvcBaud = 115200; // TODO - check baud rate for TVC
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

  if (!bus6.ready()){
    CommsSerial.println("ERROR: RS485 bus6 (USART6) init failed");
     return false;
  }

  if (!bus2.ready()){
    CommsSerial.println("ERROR: RS485 bus2 (USART2) init failed");
    return false;
  }

  return true;
}
} // namespace RS485s