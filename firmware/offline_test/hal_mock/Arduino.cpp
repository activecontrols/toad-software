#include "Arduino.h"
#include "VirtualClock.h"
#include "core/SimulatedGPIO.h"

extern "C" {

unsigned long millis(void) {
    return VirtualClock::instance().now_ms();
}

unsigned long micros(void) {
    return static_cast<unsigned long>(VirtualClock::instance().now_us());
}

void delay(unsigned long ms) {
    VirtualClock::instance().sleep_for(static_cast<uint64_t>(ms) * 1000ULL);
}

void delayMicroseconds(unsigned int us) {
    VirtualClock::instance().sleep_for(static_cast<uint64_t>(us));
}

void yield(void) {
    boost::this_fiber::yield();
}

// GPIO operations backed by SimulatedGPIO
void pinMode(pin_size_t pinNumber, PinMode mode) {
    toad::sim::SimulatedGPIO::instance().set_pin_mode(pinNumber, mode);
}

void digitalWrite(pin_size_t pinNumber, PinStatus status) {
    toad::sim::SimulatedGPIO::instance().write_pin(pinNumber, status);
}

PinStatus digitalRead(pin_size_t pinNumber) {
    return toad::sim::SimulatedGPIO::instance().read_pin(pinNumber);
}

int analogRead(pin_size_t pinNumber) {
    return toad::sim::SimulatedGPIO::instance().read_analog(pinNumber);
}

void analogWrite(pin_size_t pinNumber, int value) {
    toad::sim::SimulatedGPIO::instance().write_analog(pinNumber, value);
}

} // extern "C"
