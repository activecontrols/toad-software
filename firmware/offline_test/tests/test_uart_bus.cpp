#include <iostream>
#include <cassert>
#include <string>
#include <vector>

#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "bus/UartBus.h"
#include "bus/UartSnooper.h"
#include "hal_mock/HardwareSerial.h"
#include "runner/UartTerminalHarness.h"

using namespace toad::sim;

void test_firmware_to_bus_write() {
    std::cout << "[Test 1] Testing firmware Uart::write() to UartBus..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "RS485_TEST");
    bus->set_zero_latency(true);
    BusRegistry::instance().register_uart(1, 2, bus);

    Uart uart(1, 2);
    uart.begin(115200);

    // Firmware writes data
    uart.print("PING");
    uart.write('\n');

    assert(bus->available_from_firmware() == 5);
    std::string received;
    while (bus->available_from_firmware() > 0) {
        received += static_cast<char>(bus->read_from_firmware());
    }

    assert(received == "PING\n");
    std::cout << "  Received on bus: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

void test_bus_to_firmware_read() {
    std::cout << "[Test 2] Testing UartBus to firmware Uart::read()..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "COMM_TEST");
    bus->set_zero_latency(true);
    BusRegistry::instance().register_uart(3, 4, bus);

    Uart uart(3, 4);
    uart.begin(115200);

    assert(uart.available() == 0);

    // External entity pushes into bus
    bus->write_to_firmware("PONG\n");

    assert(uart.available() == 5);
    assert(uart.peek() == 'P');
    assert(uart.available() == 5); // peek does not consume

    std::string received;
    while (uart.available() > 0) {
        received += static_cast<char>(uart.read());
    }

    assert(received == "PONG\n");
    assert(uart.available() == 0);
    std::cout << "  Firmware read: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

void test_sim_peeking() {
    std::cout << "[Test 3] Testing non-destructive simulation peeking..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "PEEK_TEST");
    bus->set_zero_latency(true);
    BusRegistry::instance().register_uart(5, 6, bus);

    Uart uart(5, 6);
    uart.begin(115200);

    const uint8_t sample_data[] = {0x11, 0x22, 0x33, 0x44};
    uart.write(sample_data, sizeof(sample_data));

    // Sim inspects without popping
    assert(bus->peek_fw_tx() == 0x11);
    auto snapshot = bus->peek_all_fw_tx();
    assert(snapshot.size() == 4);
    assert(snapshot[0] == 0x11);
    assert(snapshot[3] == 0x44);

    // Queue size is still 4!
    assert(bus->available_from_firmware() == 4);

    // History contains the transaction
    auto history = bus->get_transaction_history();
    assert(history.size() == 1);
    assert(history[0].direction == BusDirection::FW_TO_BUS);
    assert(history[0].data.size() == 4);

    std::cout << "  Peeked 4 bytes without consuming queue." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_baud_rate_timing() {
    std::cout << "[Test 4] Testing baud rate transmission timing..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    // At 9600 baud, 10 bytes = 100 bits = (100 * 1,000,000) / 9600 = 10,416 us
    auto bus = std::make_shared<UartBus>(9600, "TIMING_BUS");
    bus->set_zero_latency(false);
    BusRegistry::instance().register_uart(7, 8, bus);

    Uart uart(7, 8);
    uart.begin(9600);

    uint64_t start_us = 0;
    uint64_t end_us = 0;

    auto f = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        start_us = VirtualClock::instance().now_us();
        uart.write(reinterpret_cast<const uint8_t*>("0123456789"), 10);
        end_us = VirtualClock::instance().now_us();
    });

    VirtualClock::instance().run_until(20000);
    f.join();

    assert(start_us == 0);
    assert(end_us == 10416);
    std::cout << "  10 bytes at 9600 baud took exactly " << end_us << " virtual microseconds." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_snooper_formatting() {
    std::cout << "[Test 5] Testing UartSnooper formats (ASCII, HEX, HEX_DUMP)..." << std::endl;

    UartTransaction tx;
    tx.timestamp_us = 1250000;
    tx.direction = BusDirection::FW_TO_BUS;
    std::string msg = "Hello 123!\r\n";
    tx.data.assign(msg.begin(), msg.end());

    std::string ascii_out = UartSnooper::format_ascii(tx, "TEST_BUS");
    std::cout << "--- ASCII Format ---\n" << ascii_out;
    assert(ascii_out.find("Hello 123!") != std::string::npos);
    assert(ascii_out.find("TEST_BUS") != std::string::npos);

    std::string hex_out = UartSnooper::format_hex(tx, "TEST_BUS");
    std::cout << "--- HEX Format ---\n" << hex_out;
    assert(hex_out.find("0x48") != std::string::npos); // 'H'
    assert(hex_out.find("0x65") != std::string::npos); // 'e'

    std::string hexdump_out = UartSnooper::format_hexdump(tx, "TEST_BUS");
    std::cout << "--- HEX DUMP Format ---\n" << hexdump_out;
    assert(hexdump_out.find("|Hello 123!") != std::string::npos);

    std::cout << "  -> PASSED" << std::endl;
}

void test_terminal_harness_injection() {
    std::cout << "[Test 6] Testing UartTerminalHarness injection into UartBus..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "HARNESS_BUS");
    bus->set_zero_latency(true);

    Uart uart(15, 16);
    uart.attach_bus(bus);
    uart.begin(115200);

    UartTerminalHarness harness(bus, SnoopFormat::FORMAT_ASCII);

    // Inject from terminal
    harness.inject_input("status\n");

    assert(uart.available() == 7);
    std::string received;
    while (uart.available() > 0) {
        received += static_cast<char>(uart.read());
    }
    assert(received == "status\n");
    std::cout << "  Injected string verified: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>

void test_uart_snooper_bidirectional_input() {
    std::cout << "[Test 7] Testing UartSnooper bidirectional PTY input relay..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "INTERACTIVE_BUS");
    bus->set_zero_latency(true);

    // Create snooper with enable_input = true and spawn_terminal = false
    auto snooper = std::make_shared<UartSnooper>(
        "TEST_INTERACTIVE", SnoopFormat::FORMAT_ASCII, nullptr, false, /*enable_input=*/true
    );
    bus->add_observer(snooper);

    assert(snooper->enable_input() == true);
    assert(snooper->bus() == bus);

    // Write to the PTY slave device
    int sfd = open(snooper->pty_slave_path().c_str(), O_WRONLY);
    assert(sfd >= 0);
    ssize_t written = write(sfd, "ping\n", 5);
    assert(written == 5);
    close(sfd);

    // Give the background input reader thread time to read and inject
    for (int i = 0; i < 50 && bus->available_for_firmware() < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(bus->available_for_firmware() >= 5);
    std::string received;
    while (bus->available_for_firmware() > 0) {
        received += static_cast<char>(bus->read_for_firmware());
    }
    assert(received.find("ping") != std::string::npos);
    std::cout << "  Input received on bus from PTY: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

void test_uart_snooper_input_toggle() {
    std::cout << "[Test 8] Testing UartSnooper enable_input toggle..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    auto bus = std::make_shared<UartBus>(115200, "TOGGLE_BUS");
    bus->set_zero_latency(true);

    // Input disabled initially
    auto snooper = std::make_shared<UartSnooper>(
        "TEST_TOGGLE", SnoopFormat::FORMAT_ASCII, nullptr, false, /*enable_input=*/false
    );
    bus->add_observer(snooper);
    assert(snooper->enable_input() == false);

    int sfd = open(snooper->pty_slave_path().c_str(), O_WRONLY);
    assert(sfd >= 0);
    write(sfd, "ignored\n", 8);

    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    assert(bus->available_for_firmware() == 0);

    // Now enable input
    snooper->set_enable_input(true);
    assert(snooper->enable_input() == true);

    write(sfd, "accepted\n", 9);
    close(sfd);

    for (int i = 0; i < 50 && bus->available_for_firmware() < 9; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(bus->available_for_firmware() >= 9);
    std::string received;
    while (bus->available_for_firmware() > 0) {
        received += static_cast<char>(bus->read_for_firmware());
    }
    assert(received.find("accepted") != std::string::npos);
    std::cout << "  Toggled input received on bus: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    setenv("TOAD_SNOOPER_NO_GUI", "1", 1);
    std::cout << "=== Running Uart, UartBus & Snooper Verification Tests ===" << std::endl;
    install_fiber_scheduler();

    test_firmware_to_bus_write();
    test_bus_to_firmware_read();
    test_sim_peeking();
    test_baud_rate_timing();
    test_snooper_formatting();
    test_terminal_harness_injection();
    test_uart_snooper_bidirectional_input();
    test_uart_snooper_input_toggle();

    std::cout << "=== All Milestone 2 Tests Passed Successfully! ===" << std::endl;
    return 0;
}
