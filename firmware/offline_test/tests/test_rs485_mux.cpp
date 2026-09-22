#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <cmath>

#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "core/SimulatedGPIO.h"
#include "hal_mock/Arduino.h"
#include "hal_mock/pins_arduino.h"
#include "hal_mock/HardwareSerial.h"
#include "bus/RS485Mux.h"
#include "peripherals/IRS485Device.h"
#include "peripherals/AMT242AV_Sim.h"
#include "hardware_mapping/ec_pins.h"
#include "throttle_valves/AMT242AV.h"

using namespace toad::sim;

void test_gpio_tracking_and_listeners() {
    std::cout << "[Test 1] Testing SimulatedGPIO pin state tracking & listeners..." << std::endl;
    SimulatedGPIO::instance().reset();

    pinMode(PA0, arduino::OUTPUT);
    assert(SimulatedGPIO::instance().get_pin_mode(PA0) == arduino::OUTPUT);

    int callback_count = 0;
    arduino::PinStatus captured_new = arduino::LOW;
    arduino::PinStatus captured_old = arduino::HIGH;

    SimulatedGPIO::instance().add_listener(PA0, [&](uint32_t pin, arduino::PinStatus n, arduino::PinStatus o) {
        assert(pin == PA0);
        callback_count++;
        captured_new = n;
        captured_old = o;
    });

    // Write HIGH
    digitalWrite(PA0, arduino::HIGH);
    assert(digitalRead(PA0) == arduino::HIGH);
    assert(callback_count == 1);
    assert(captured_new == arduino::HIGH);
    assert(captured_old == arduino::LOW);

    // Writing same value should not trigger listener
    digitalWrite(PA0, arduino::HIGH);
    assert(callback_count == 1);

    // Write LOW
    digitalWrite(PA0, arduino::LOW);
    assert(digitalRead(PA0) == arduino::LOW);
    assert(callback_count == 2);
    assert(captured_new == arduino::LOW);
    assert(captured_old == arduino::HIGH);

    std::cout << "  -> PASSED" << std::endl;
}

void test_rs485_mux_single_device() {
    std::cout << "[Test 2] Testing RS485Mux single device routing on SEL line..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_TEST");
    mux->set_zero_latency(true);

    std::vector<uint8_t> received_bytes;
    auto mock_dev = std::make_shared<FunctionalRS485Device>("EchoDevice",
        [&](const uint8_t* buf, size_t size, IRS485Device& dev) {
            received_bytes.assign(buf, buf + size);
            // Echo back with prefix
            std::string reply = "ECHO:";
            reply.append(reinterpret_cast<const char*>(buf), size);
            dev.transmit_to_bus(reply);
        });

    mux->register_device(PIN_ENC_OX_SEL, mock_dev);

    // Initially SEL is LOW (unselected)
    assert(digitalRead(PIN_ENC_OX_SEL) == arduino::LOW);
    assert(mux->active_sel_pin() == NC);
    assert(mux->active_device() == nullptr);

    // Firmware sends data while unselected -> device should NOT receive it
    const char* ping = "PING";
    mux->write_from_firmware(reinterpret_cast<const uint8_t*>(ping), 4);
    assert(received_bytes.empty());
    assert(mux->available_for_firmware() == 0);

    // Now drive SEL line HIGH
    digitalWrite(PIN_ENC_OX_SEL, arduino::HIGH);
    assert(mux->active_sel_pin() == PIN_ENC_OX_SEL);
    assert(mux->active_device() == mock_dev);

    // Firmware writes data -> device should receive it and respond
    mux->write_from_firmware(reinterpret_cast<const uint8_t*>(ping), 4);
    assert(received_bytes.size() == 4);
    assert(std::string(received_bytes.begin(), received_bytes.end()) == "PING");

    // Check reply in firmware receive queue
    assert(mux->available_for_firmware() == 9); // "ECHO:PING"
    std::string fw_reply;
    while (mux->available_for_firmware() > 0) {
        fw_reply += static_cast<char>(mux->read_for_firmware());
    }
    assert(fw_reply == "ECHO:PING");

    // Deselect
    digitalWrite(PIN_ENC_OX_SEL, arduino::LOW);
    assert(mux->active_sel_pin() == NC);
    assert(mux->active_device() == nullptr);

    // If device tries to transmit while unselected, mux blocks it
    mock_dev->transmit_to_bus("ORPHAN");
    assert(mux->available_for_firmware() == 0);

    std::cout << "  -> PASSED" << std::endl;
}

void test_multiple_devices_muxing_isolation() {
    std::cout << "[Test 3] Testing multiple devices routing & bus isolation..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_6");
    mux->set_zero_latency(true);

    std::string dev_a_data;
    auto dev_a = std::make_shared<FunctionalRS485Device>("OxEncoder",
        [&](const uint8_t* buf, size_t sz, IRS485Device& dev) {
            dev_a_data.assign(reinterpret_cast<const char*>(buf), sz);
            dev.transmit_to_bus("RESP_OX");
        });

    std::string dev_b_data;
    auto dev_b = std::make_shared<FunctionalRS485Device>("PitchActuator",
        [&](const uint8_t* buf, size_t sz, IRS485Device& dev) {
            dev_b_data.assign(reinterpret_cast<const char*>(buf), sz);
            dev.transmit_to_bus("RESP_TVC");
        });

    mux->register_device(PIN_ENC_OX_SEL, dev_a);
    mux->register_device(PIN_TVC_PITCH_SEL, dev_b);

    // 1. Select Device A
    digitalWrite(PIN_ENC_OX_SEL, arduino::HIGH);
    digitalWrite(PIN_TVC_PITCH_SEL, arduino::LOW);
    assert(mux->active_device() == dev_a);

    mux->write_from_firmware(reinterpret_cast<const uint8_t*>("CMD_TO_OX"), 9);
    assert(dev_a_data == "CMD_TO_OX");
    assert(dev_b_data.empty()); // Dev B isolated

    std::string reply_a;
    while (mux->available_for_firmware() > 0) {
        reply_a += static_cast<char>(mux->read_for_firmware());
    }
    assert(reply_a == "RESP_OX");

    // 2. Switch to Device B
    digitalWrite(PIN_ENC_OX_SEL, arduino::LOW);
    digitalWrite(PIN_TVC_PITCH_SEL, arduino::HIGH);
    assert(mux->active_device() == dev_b);

    mux->write_from_firmware(reinterpret_cast<const uint8_t*>("CMD_TO_TVC"), 10);
    assert(dev_b_data == "CMD_TO_TVC");
    assert(dev_a_data == "CMD_TO_OX"); // Dev A saw no new data

    std::string reply_b;
    while (mux->available_for_firmware() > 0) {
        reply_b += static_cast<char>(mux->read_for_firmware());
    }
    assert(reply_b == "RESP_TVC");

    std::cout << "  -> PASSED" << std::endl;
}

void test_bus_contention_detection() {
    std::cout << "[Test 4] Testing bus contention detection when multiple SEL lines are HIGH..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_6");
    mux->set_zero_latency(true);

    int dev_a_packets = 0;
    auto dev_a = std::make_shared<FunctionalRS485Device>("DevA",
        [&](const uint8_t*, size_t, IRS485Device& dev) {
            dev_a_packets++;
            dev.transmit_to_bus("A");
        });

    int dev_b_packets = 0;
    auto dev_b = std::make_shared<FunctionalRS485Device>("DevB",
        [&](const uint8_t*, size_t, IRS485Device& dev) {
            dev_b_packets++;
            dev.transmit_to_bus("B");
        });

    mux->register_device(PIN_ENC_OX_SEL, dev_a);
    mux->register_device(PIN_TVC_PITCH_SEL, dev_b);

    // Drive BOTH select lines HIGH simultaneously (invalid bus state)
    digitalWrite(PIN_ENC_OX_SEL, arduino::HIGH);
    digitalWrite(PIN_TVC_PITCH_SEL, arduino::HIGH);

    assert(mux->has_bus_contention() == true);
    assert(mux->contention_count() == 1);
    assert(mux->active_device() == nullptr);

    // Firmware transmission during contention must be dropped
    mux->write_from_firmware(reinterpret_cast<const uint8_t*>("COLLIDE"), 7);
    assert(dev_a_packets == 0);
    assert(dev_b_packets == 0);
    assert(mux->available_for_firmware() == 0);

    // Device transmission during contention must be dropped
    dev_a->transmit_to_bus("DROP_ME");
    assert(mux->available_for_firmware() == 0);

    // Resolve contention: pull PIN_TVC_PITCH_SEL LOW
    digitalWrite(PIN_TVC_PITCH_SEL, arduino::LOW);
    assert(mux->has_bus_contention() == false);
    assert(mux->contention_count() == 1); // Historical count preserved
    assert(mux->active_device() == dev_a);

    // Normal transmission now succeeds
    mux->write_from_firmware(reinterpret_cast<const uint8_t*>("OK"), 2);
    assert(dev_a_packets == 1);
    assert(mux->available_for_firmware() == 1);

    std::cout << "  -> PASSED" << std::endl;
}

void test_pluggable_substitution() {
    std::cout << "[Test 5] Testing custom peripheral model substitution..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_CUSTOM");
    mux->set_zero_latency(true);

    // Custom motor driver model with custom telemetry
    struct MotorState {
        int target_rpm{0};
        int current_rpm{0};
    } motor;

    auto custom_motor = std::make_shared<FunctionalRS485Device>("CustomMotorModel",
        [&motor](const uint8_t* buf, size_t sz, IRS485Device& dev) {
            if (sz >= 3 && buf[0] == 0x55) { // Command packet
                motor.target_rpm = (buf[1] << 8) | buf[2];
                motor.current_rpm = motor.target_rpm; // instant mock response
                uint8_t telemetry[4] = {
                    0xAA,
                    buf[0],
                    static_cast<uint8_t>((motor.current_rpm >> 8) & 0xFF),
                    static_cast<uint8_t>(motor.current_rpm & 0xFF)
                };
                dev.transmit_to_bus(telemetry, 4);
            }
        });

    mux->register_device(PIN_TVC_YAW_SEL, custom_motor);

    digitalWrite(PIN_TVC_YAW_SEL, arduino::HIGH);

    uint8_t cmd[3] = { 0x55, 0x07, 0xD0 }; // Set 2000 RPM (0x07D0)
    mux->write_from_firmware(cmd, 3);

    assert(mux->available_for_firmware() == 4);
    uint8_t resp[4];
    for (int i = 0; i < 4; ++i) {
        resp[i] = static_cast<uint8_t>(mux->read_for_firmware());
    }

    assert(resp[0] == 0xAA);
    assert(resp[1] == 0x55);
    assert(resp[2] == 0x07);
    assert(resp[3] == 0xD0);
    assert(motor.current_rpm == 2000);

    digitalWrite(PIN_TVC_YAW_SEL, arduino::LOW);

    std::cout << "  -> PASSED" << std::endl;
}

void test_amt242av_encoder_model() {
    std::cout << "[Test 6] Testing AMT242AV encoder simulation model & parity..." << std::endl;
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_ENC");
    mux->set_zero_latency(true);

    auto encoder = std::make_shared<AMT242AV_Sim>(0x00, "OxShaftEncoder");
    mux->register_device(PIN_ENC_OX_SEL, encoder);

    // Set position to 2730 (0x0AAA)
    encoder->set_raw_position(0x0AAA);
    assert(encoder->get_raw_position() == 0x0AAA);

    digitalWrite(PIN_ENC_OX_SEL, arduino::HIGH);

    // Send read command (ID = 0x00)
    uint8_t cmd = 0x00;
    mux->write_from_firmware(&cmd, 1);

    assert(mux->available_for_firmware() == 2);
    uint8_t b0 = static_cast<uint8_t>(mux->read_for_firmware());
    uint8_t b1 = static_cast<uint8_t>(mux->read_for_firmware());
    uint16_t response = static_cast<uint16_t>(b0) | (static_cast<uint16_t>(b1) << 8);

    // Validate 12-bit position in response (bits 2..13)
    uint16_t decoded_pos = (response >> 2) & 0x0FFF;
    assert(decoded_pos == 0x0AAA);

    // Validate 2-bit odd parity checksum in response (bits 14..15)
    uint8_t decoded_cs = (response >> 14) & 0x03;
    uint8_t expected_cs = AMT242AV_Sim::calculate_checksum(0x0AAA);
    assert(decoded_cs == expected_cs);

    // Test fraction setter/getter
    encoder->set_position_fraction(0.5f);
    assert(encoder->get_raw_position() == 2047);

    // Test zero command (ID | 0x02) - sets zero reference position
    uint8_t zero_cmd = 0x02;
    mux->write_from_firmware(&zero_cmd, 1);
    assert(encoder->get_raw_position() == 0);

    // Set position to a non-zero value
    encoder->set_raw_position(1500);

    // Test reset command (ID | 0x03) - reboots controller, does NOT zero position
    encoder->set_reset_delay_us(0); // Zero latency for sync unit test
    uint8_t reset_cmd = 0x03;
    mux->write_from_firmware(&reset_cmd, 1);
    assert(encoder->reset_count() == 1);
    assert(encoder->get_raw_position() == 1500); // Absolute position preserved!

    digitalWrite(PIN_ENC_OX_SEL, arduino::LOW);

    std::cout << "  -> PASSED" << std::endl;
}

void test_firmware_uart_class_integration() {
    std::cout << "[Test 7] Testing HardwareSerial/Uart class integration with RS485Mux..." << std::endl;
    SimulatedGPIO::instance().reset();
    BusRegistry::instance().reset();

    // Create mux and register into BusRegistry
    auto mux = std::make_shared<RS485Mux>(115200, "RS485_6", PIN_RS485_6_DE);
    mux->set_zero_latency(true);
    BusRegistry::instance().register_uart(PIN_RS485_6_RX, PIN_RS485_6_TX, mux);

    // Attach encoder to mux
    auto encoder = std::make_shared<AMT242AV_Sim>(0x00, "OxEncoder");
    encoder->set_raw_position(1234);
    mux->register_device(PIN_ENC_OX_SEL, encoder);

    // Emulate firmware Uart instance
    Uart uart(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE);
    uart.begin(115200);

    // Configure SEL pin as output (like AMT242AV::begin())
    pinMode(PIN_ENC_OX_SEL, arduino::OUTPUT);
    digitalWrite(PIN_ENC_OX_SEL, arduino::LOW);

    // Emulate firmware AMT242AV::_read_pos()
    digitalWrite(PIN_ENC_OX_SEL, arduino::HIGH);
    uart.write(static_cast<uint8_t>(0x00)); // ID
    uart.flush();

    assert(uart.available() == 2);
    uint16_t res = 0;
    res |= uart.read();
    res |= (uart.read() << 8);

    digitalWrite(PIN_ENC_OX_SEL, arduino::LOW);

    uint16_t pos = (res >> 2) & 0x0FFF;
    assert(pos == 1234);

    std::cout << "  Firmware successfully read 12-bit encoder position: " << pos << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_amt242av_fiber_channel_concurrency() {
    std::cout << "[Test 8] Testing Pattern 2 AMT242AV fiber queue concurrency with production driver..." << std::endl;
    VirtualClock::instance().reset(0);
    SimulatedGPIO::instance().reset();
    BusRegistry::instance().reset();

    // 1. Setup mux and register into BusRegistry
    auto mux = std::make_shared<RS485Mux>(115200, "RS485_6", PIN_RS485_6_DE);
    mux->set_zero_latency(true);
    BusRegistry::instance().register_uart(PIN_RS485_6_RX, PIN_RS485_6_TX, mux);

    // 2. Setup encoder with independent fiber
    auto encoder_sim = std::make_shared<AMT242AV_Sim>(0x00, "OxShaftSim");
    encoder_sim->set_raw_position(3000); // 3000 raw ticks
    encoder_sim->set_response_delay_us(70); // 70 us physical response latency
    encoder_sim->set_reset_delay_us(5000);  // 5000 us (5 ms) reset delay
    mux->register_device(PIN_ENC_OX_SEL, encoder_sim);

    // Launch peripheral on its own fiber (Priority 5)
    encoder_sim->start(PRIO_SENSORS);
    assert(encoder_sim->is_running());

    // 3. Firmware fiber using actual production driver AMT242AV
    Uart uart(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE);
    uart.begin(115200);

    bool test_passed = false;
    float read_angle = 0.0f;

    auto f_fw = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        // Instantiate the REAL production driver from firmware/lib/throttle_valves/AMT242AV.h
        AMT242AV driver(uart, PIN_ENC_OX_SEL, 0x00);
        driver.begin();

        // 3a. Read position via driver
        bool ok = driver.read_pos(&read_angle);
        assert(ok);

        // Expected fraction: 3000 / 4095 = ~0.7326 (read_pos returns fraction 0.0 to 1.0)
        float expected_frac = 3000.0f / 4095.0f;
        assert(std::abs(read_angle - expected_frac) < 0.001f);

        // 3b. Test zeroing via production driver
        driver.zero();
        delayMicroseconds(100); // Allow peripheral fiber to process zero command
        assert(encoder_sim->get_raw_position() == 0);

        // Read back after zero
        float zero_frac = 0.0f;
        ok = driver.read_pos(&zero_frac);
        assert(ok);
        assert(std::abs(zero_frac - 0.0f) < 0.001f);

        // 3c. Set position and test reset via production driver
        encoder_sim->set_raw_position(1800);
        driver.reset();
        delay(10); // Wait for the 5 ms controller reboot in virtual time
        assert(encoder_sim->reset_count() == 1);
        assert(encoder_sim->get_raw_position() == 1800); // Position retained across reset

        test_passed = true;
    });

    // Run simulation until all work completes (e.g. 50 ms virtual time)
    VirtualClock::instance().run_until(50000);
    f_fw.join();

    // Cleanly stop peripheral fiber
    encoder_sim->stop();
    assert(!encoder_sim->is_running());

    assert(test_passed);
    std::cout << "  Read angle fraction: " << read_angle << " from production driver across fiber boundary." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_functional_device_fiber_channel() {
    std::cout << "[Test 9] Testing FunctionalRS485Device with independent fiber & message queue..." << std::endl;
    VirtualClock::instance().reset(0);
    SimulatedGPIO::instance().reset();

    auto mux = std::make_shared<RS485Mux>(115200, "RS485_FUNC");
    mux->set_zero_latency(true);

    std::vector<uint8_t> received_packet;
    int rx_count = 0;

    auto dev = std::make_shared<FunctionalRS485Device>("AsyncMotorMock",
        [&](const uint8_t* buf, size_t sz, IRS485Device& d) {
            received_packet.assign(buf, buf + sz);
            rx_count++;
            // Simulate 50 us processing time
            VirtualClock::instance().sleep_for(50);
            d.transmit_to_bus("ACK");
        });

    mux->register_device(PIN_TVC_PITCH_SEL, dev);
    dev->start(PRIO_SENSORS);
    assert(dev->is_running());

    bool fw_completed = false;
    auto f_fw = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        digitalWrite(PIN_TVC_PITCH_SEL, arduino::HIGH);
        const char* cmd = "GOTO_POS";
        mux->write_from_firmware(reinterpret_cast<const uint8_t*>(cmd), 8);

        // At this instant, packet is in rx_channel_, dev hasn't replied yet
        assert(mux->available_for_firmware() == 0);

        // Firmware waits/delays 60 us
        delayMicroseconds(60);

        // Peripheral fiber should have woken, slept 50 us, and transmitted "ACK"
        assert(mux->available_for_firmware() == 3);
        uint8_t reply[3];
        reply[0] = static_cast<uint8_t>(mux->read_for_firmware());
        reply[1] = static_cast<uint8_t>(mux->read_for_firmware());
        reply[2] = static_cast<uint8_t>(mux->read_for_firmware());
        assert(std::string(reinterpret_cast<char*>(reply), 3) == "ACK");

        digitalWrite(PIN_TVC_PITCH_SEL, arduino::LOW);
        fw_completed = true;
    });

    VirtualClock::instance().run_until(500);
    f_fw.join();
    dev->stop();
    assert(!dev->is_running());

    assert(fw_completed);
    assert(rx_count == 1);
    assert(std::string(received_packet.begin(), received_packet.end()) == "GOTO_POS");
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    install_fiber_scheduler();
    std::cout << "=== Running RS485Mux & Pluggable Peripherals Tests ===" << std::endl;

    test_gpio_tracking_and_listeners();
    test_rs485_mux_single_device();
    test_multiple_devices_muxing_isolation();
    test_bus_contention_detection();
    test_pluggable_substitution();
    test_amt242av_encoder_model();
    test_firmware_uart_class_integration();
    test_amt242av_fiber_channel_concurrency();
    test_functional_device_fiber_channel();

    std::cout << "=== All RS485Mux & Pluggable Peripherals Tests Passed Successfully! ===" << std::endl;
    return 0;
}
