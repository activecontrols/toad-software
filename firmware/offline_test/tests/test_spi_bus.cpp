#include <iostream>
#include <cassert>
#include <string>
#include <vector>

#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "core/SimulatedGPIO.h"
#include "hal_mock/Arduino.h"
#include "hal_mock/pins_arduino.h"
#include "hal_mock/SPI.h"
#include "bus/SPIBus.h"
#include "peripherals/ISPIDevice.h"
#include "hardware_mapping/ec_pins.h"

using namespace toad::sim;

void test_cs_tracking_and_callbacks() {
    std::cout << "[Test 1] Testing SPI CS state tracking & assertion callbacks..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("TEST_SPI", PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
    bus->set_zero_latency(true);

    int assert_count = 0;
    int deassert_count = 0;

    auto dev = std::make_shared<FunctionalSPIDevice>("MockADC",
        [](uint8_t mosi, FunctionalSPIDevice&) -> uint8_t {
            return mosi ^ 0xFF;
        },
        [&](FunctionalSPIDevice&) { assert_count++; },
        [&](FunctionalSPIDevice&) { deassert_count++; }
    );

    // Initial state: CS pin is HIGH (inactive for active-low)
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    bus->register_device(PIN_PT_BOARD_1_2_CS, dev, /*active_low=*/true);

    assert(bus->active_cs_pin() == NC);
    assert(bus->active_device() == nullptr);
    assert(assert_count == 0);
    assert(deassert_count == 0);

    // Assert CS (drive LOW)
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
    assert(bus->active_cs_pin() == PIN_PT_BOARD_1_2_CS);
    assert(bus->active_device() == dev);
    assert(assert_count == 1);
    assert(deassert_count == 0);

    // Deassert CS (drive HIGH)
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    assert(bus->active_cs_pin() == NC);
    assert(bus->active_device() == nullptr);
    assert(assert_count == 1);
    assert(deassert_count == 1);

    std::cout << "  -> PASSED" << std::endl;
}

void test_single_device_full_duplex() {
    std::cout << "[Test 2] Testing SPI full-duplex MOSI/MISO byte transfer..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("TEST_SPI");
    bus->set_zero_latency(true);

    auto dev = std::make_shared<FunctionalSPIDevice>("InverterDevice",
        [](uint8_t mosi, FunctionalSPIDevice&) -> uint8_t {
            return static_cast<uint8_t>(~mosi & 0xFF);
        });

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    bus->register_device(PIN_PT_BOARD_1_2_CS, dev);

    // When unselected, transfer should return 0xFF
    uint8_t unselected_rx = bus->transfer(0xAA);
    assert(unselected_rx == 0xFF);

    // Assert CS
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);

    // Transfer bytes
    assert(bus->transfer(0xA5) == 0x5A);
    assert(bus->transfer(0x12) == 0xED);
    assert(bus->transfer(0x00) == 0xFF);
    assert(bus->transfer(0xFF) == 0x00);

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    std::cout << "  -> PASSED" << std::endl;
}

void test_buffer_and_word_transfer() {
    std::cout << "[Test 3] Testing buffer & transfer16 transfers..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("TEST_SPI");
    bus->set_zero_latency(true);

    auto dev = std::make_shared<FunctionalSPIDevice>("AdderDevice",
        [](uint8_t mosi, FunctionalSPIDevice&) -> uint8_t {
            return static_cast<uint8_t>(mosi + 1);
        });

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    bus->register_device(PIN_PT_BOARD_1_2_CS, dev);

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
    bus->begin_transaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    // Test transfer16
    uint16_t resp16 = bus->transfer16(0x0102);
    assert(resp16 == 0x0203);

    // Test in-place buffer transfer
    uint8_t buffer[4] = {0x10, 0x20, 0x30, 0x40};
    bus->transfer(buffer, 4);
    assert(buffer[0] == 0x11);
    assert(buffer[1] == 0x21);
    assert(buffer[2] == 0x31);
    assert(buffer[3] == 0x41);

    bus->end_transaction();
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);

    std::cout << "  -> PASSED" << std::endl;
}

void test_multiple_devices_cs_isolation() {
    std::cout << "[Test 4] Testing multiple SPI devices & CS isolation..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("PT_TC_SPI_1");
    bus->set_zero_latency(true);

    auto dev_a = std::make_shared<FunctionalSPIDevice>("ADC_1_2",
        [](uint8_t, FunctionalSPIDevice&) -> uint8_t {
            return 0x11;
        });

    auto dev_b = std::make_shared<FunctionalSPIDevice>("ADC_3_4",
        [](uint8_t, FunctionalSPIDevice&) -> uint8_t {
            return 0x22;
        });

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::HIGH);

    bus->register_device(PIN_PT_BOARD_1_2_CS, dev_a);
    bus->register_device(PIN_PT_BOARD_3_4_CS, dev_b);

    // 1. Select Device A
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::HIGH);
    assert(bus->active_device() == dev_a);
    assert(bus->transfer(0x00) == 0x11);

    // 2. Select Device B
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::LOW);
    assert(bus->active_device() == dev_b);
    assert(bus->transfer(0x00) == 0x22);

    // 3. Deselect both
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::HIGH);
    assert(bus->active_device() == nullptr);
    assert(bus->transfer(0x00) == 0xFF);

    std::cout << "  -> PASSED" << std::endl;
}

void test_bus_contention() {
    std::cout << "[Test 5] Testing bus contention when multiple CS lines are asserted LOW..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("BUS_CONTENTION_TEST");
    bus->set_zero_latency(true);

    auto dev_a = std::make_shared<FunctionalSPIDevice>("DevA", [](uint8_t, FunctionalSPIDevice&) { return 0xAA; });
    auto dev_b = std::make_shared<FunctionalSPIDevice>("DevB", [](uint8_t, FunctionalSPIDevice&) { return 0xBB; });

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::HIGH);

    bus->register_device(PIN_PT_BOARD_1_2_CS, dev_a);
    bus->register_device(PIN_PT_BOARD_3_4_CS, dev_b);

    // Assert BOTH CS pins simultaneously (illegal hardware state)
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::LOW);

    assert(bus->has_bus_contention() == true);
    assert(bus->contention_count() == 1);
    assert(bus->active_device() == nullptr);

    // Transfer returns floating 0xFF collision
    assert(bus->transfer(0x00) == 0xFF);

    // Resolve contention: deassert Dev B
    digitalWrite(PIN_PT_BOARD_3_4_CS, arduino::HIGH);
    assert(bus->has_bus_contention() == false);
    assert(bus->active_device() == dev_a);
    assert(bus->transfer(0x00) == 0xAA);

    std::cout << "  -> PASSED" << std::endl;
}

void test_spi_settings_clock_timing() {
    std::cout << "[Test 6] Testing SPISettings clock frequency simulation in VirtualClock..." << std::endl;
    VirtualClock::instance().reset(0);
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("TIMING_SPI");
    bus->set_zero_latency(false);

    auto dev = std::make_shared<FunctionalSPIDevice>("FastDev", [](uint8_t m, FunctionalSPIDevice&) { return m; });
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    bus->register_device(PIN_PT_BOARD_1_2_CS, dev);

    // At 1 MHz (1,000,000 Hz), 1 byte (8 bits) = 8 us
    SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

    uint64_t start_us = 0;
    uint64_t end_us = 0;

    auto f = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
        bus->begin_transaction(settings);
        start_us = VirtualClock::instance().now_us();

        // Transfer 10 bytes = 80 us
        for (int i = 0; i < 10; ++i) {
            bus->transfer(static_cast<uint8_t>(i));
        }

        end_us = VirtualClock::instance().now_us();
        bus->end_transaction();
        digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);
    });

    VirtualClock::instance().run_until(200);
    f.join();

    assert(end_us - start_us == 80);
    std::cout << "  10 bytes at 1 MHz took exactly " << (end_us - start_us) << " virtual us." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_spi_class_arduino_api_and_copy_semantics() {
    std::cout << "[Test 7] Testing concrete SPIClass with Arduino API & copy-by-value semantics..." << std::endl;
    VirtualClock::instance().reset(0);
    SimulatedGPIO::instance().reset();
    BusRegistry::instance().reset();

    // Create bus and register in BusRegistry
    auto bus = std::make_shared<SPIBus>("PT_TC_SPI_1", PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
    bus->set_zero_latency(true);
    BusRegistry::instance().register_spi(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK, bus);

    auto adc_model = std::make_shared<FunctionalSPIDevice>("ADS131M02_Mock",
        [](uint8_t mosi, FunctionalSPIDevice&) -> uint8_t {
            if (mosi == 0xAA) return 0x12;
            return 0x34;
        });
    bus->register_device(PIN_PT_BOARD_1_2_CS, adc_model);

    // Initialize SPIClass (as in ec_main.cpp)
    SPIClass spi1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
    spi1.begin();

    // Verify copy-by-value constructor (as in ADS131M02(SPIClass spi_bus, ...))
    SPIClass spi_copy = spi1;

    // Emulate firmware transaction via spi_copy
    pinMode(PIN_PT_BOARD_1_2_CS, arduino::OUTPUT);
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);

    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::LOW);
    spi_copy.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

    uint8_t rx1 = spi_copy.transfer(0xAA);
    uint8_t rx2 = spi_copy.transfer(0x00);

    spi_copy.endTransaction();
    digitalWrite(PIN_PT_BOARD_1_2_CS, arduino::HIGH);

    assert(rx1 == 0x12);
    assert(rx2 == 0x34);

    std::cout << "  Firmware read simulated ADC bytes [0x12, 0x34] via copied SPIClass." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_spi_transaction_observer_snooper() {
    std::cout << "[Test 8] Testing SPI transaction history & observer notifications..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto bus = std::make_shared<SPIBus>("OBS_SPI");
    bus->set_zero_latency(true);

    auto dev = std::make_shared<FunctionalSPIDevice>("Thermocouple",
        [](uint8_t, FunctionalSPIDevice&) { return 0x7E; });
    bus->register_device(PIN_TC_CHIP_1_CS, dev);

    struct MockSpiObserver : public ISpiObserver {
        int tx_count{0};
        SpiTransaction last_tx;
        void on_spi_transaction(const SpiTransaction& tx) override {
            tx_count++;
            last_tx = tx;
        }
    };

    auto observer = std::make_shared<MockSpiObserver>();
    bus->add_observer(observer);

    digitalWrite(PIN_TC_CHIP_1_CS, arduino::LOW);
    bus->begin_transaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    bus->transfer(0x80);
    bus->transfer(0x01);
    bus->end_transaction();
    digitalWrite(PIN_TC_CHIP_1_CS, arduino::HIGH);

    assert(observer->tx_count == 1);
    assert(observer->last_tx.cs_pin == PIN_TC_CHIP_1_CS);
    assert(observer->last_tx.device_name == "Thermocouple");
    assert(observer->last_tx.mosi_data.size() == 2);
    assert(observer->last_tx.mosi_data[0] == 0x80);
    assert(observer->last_tx.mosi_data[1] == 0x01);
    assert(observer->last_tx.miso_data[0] == 0x7E);
    assert(observer->last_tx.miso_data[1] == 0x7E);

    auto history = bus->get_transaction_history();
    assert(history.size() == 1);

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=== Running SPIBus & Pluggable Peripherals Tests ===" << std::endl;
    install_fiber_scheduler();

    test_cs_tracking_and_callbacks();
    test_single_device_full_duplex();
    test_buffer_and_word_transfer();
    test_multiple_devices_cs_isolation();
    test_bus_contention();
    test_spi_settings_clock_timing();
    test_spi_class_arduino_api_and_copy_semantics();
    test_spi_transaction_observer_snooper();

    std::cout << "=== All SPIBus & Pluggable Peripherals Tests Passed Successfully! ===" << std::endl;
    return 0;
}
