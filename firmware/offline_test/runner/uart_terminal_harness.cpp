#include <iostream>
#include <string>
#include <csignal>
#include <chrono>
#include <thread>

#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "bus/UartBus.h"
#include "bus/UartSnooper.h"
#include "hal_mock/HardwareSerial.h"
#include "UartTerminalHarness.h"

using namespace toad::sim;

static std::atomic<bool> g_shutdown{false};

void sigint_handler(int sig) {
    (void)sig;
    g_shutdown.store(true);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigint_handler);

    SnoopFormat format = SnoopFormat::FORMAT_ASCII;
    bool use_pty = false;
    unsigned long baud = 115200;
    bool echo_firmware = true;
    std::string bus_name = "RS485_6";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--format" && i + 1 < argc) {
            std::string fmt_str = argv[++i];
            if (fmt_str == "hex") format = SnoopFormat::FORMAT_HEX;
            else if (fmt_str == "hexdump") format = SnoopFormat::FORMAT_HEX_DUMP;
            else format = SnoopFormat::FORMAT_ASCII;
        } else if (arg == "--pty") {
            use_pty = true;
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoul(argv[++i]);
        } else if (arg == "--no-echo") {
            echo_firmware = false;
        } else if (arg == "--bus" && i + 1 < argc) {
            bus_name = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --format <ascii|hex|hexdump>  Output snooper format (default: ascii)\n"
                      << "  --pty                         Open a dedicated pseudo-terminal (/dev/pts/X)\n"
                      << "  --baud <rate>                 Baud rate (default: 115200)\n"
                      << "  --bus <name>                  Bus name (default: RS485_6)\n"
                      << "  --no-echo                     Disable automatic firmware mock echo\n"
                      << "  --help, -h                    Show this help\n";
            return 0;
        }
    }

    std::cout << "=== Toad UART Terminal Harness ===" << std::endl;
    std::cout << "Bus: " << bus_name << " @ " << baud << " baud" << std::endl;

    install_fiber_scheduler();
    VirtualClock::instance().reset(0);

    // Create or retrieve bus
    auto bus = std::make_shared<UartBus>(baud, bus_name);
    BusRegistry::instance().register_named_uart(bus_name, bus);
    // Pin pair (e.g. 10, 11)
    BusRegistry::instance().register_uart(10, 11, bus);

    Uart mock_uart(10, 11);
    mock_uart.begin(baud);

    // Create terminal harness
    UartTerminalHarness harness(bus, format);

    if (use_pty) {
        std::string pty_path = harness.enable_pty();
        if (pty_path.empty()) {
            std::cerr << "Failed to allocate PTY!" << std::endl;
            return 1;
        }
        std::cout << "PTY allocated: " << pty_path << std::endl;
        std::cout << "Connect from another terminal via:\n"
                  << "  screen " << pty_path << "\n"
                  << "  or: minicom -D " << pty_path << "\n"
                  << "  or: cat " << pty_path << std::endl;
    } else {
        std::cout << "Interactive Console Active (Type commands, press Enter; Ctrl+C to exit):" << std::endl;
    }

    harness.start_interactive();

    // Launch firmware mock fiber that reads from mock_uart and responds
    boost::fibers::fiber fw_fiber;
    if (echo_firmware) {
        fw_fiber = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
            std::string line;
            while (!g_shutdown.load()) {
                if (mock_uart.available()) {
                    char c = static_cast<char>(mock_uart.read());
                    if (c == '\r' || c == '\n') {
                        if (!line.empty()) {
                            if (line == "ping") {
                                mock_uart.println("pong");
                            } else if (line == "status") {
                                mock_uart.println("STATUS_OK: SITL EC Running");
                            } else {
                                mock_uart.print("[FW ECHO]: ");
                                mock_uart.println(line.c_str());
                            }
                            line.clear();
                        }
                    } else {
                        line += c;
                    }
                } else {
                    VirtualClock::instance().sleep_for(1000); // 1 ms polling
                }
            }
        });
    }

    // Main event loop: advances virtual time in real-time paced intervals
    auto last_wall = std::chrono::steady_clock::now();
    while (!g_shutdown.load()) {
        auto now_wall = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now_wall - last_wall).count();
        last_wall = now_wall;

        if (elapsed_us > 0) {
            VirtualClock::instance().advance_time_to(VirtualClock::instance().now_us() + elapsed_us);
        }

        if (harness.eof_reached() && bus->available_for_firmware() == 0 && !mock_uart.available()) {
            // Give firmware time to process final bytes and flush responses
            VirtualClock::instance().advance_time_to(VirtualClock::instance().now_us() + 50000);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            g_shutdown.store(true);
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\nShutting down terminal harness..." << std::endl;
    g_shutdown.store(true);
    harness.stop();

    // Advance clock so any sleeping fibers wake up and observe shutdown
    VirtualClock::instance().advance_time_to(VirtualClock::instance().now_us() + 100000);
    if (fw_fiber.joinable()) {
        fw_fiber.join();
    }

    std::cout << "Done." << std::endl;
    return 0;
}
