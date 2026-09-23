#include <iostream>
#include <cassert>
#include <string>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>

#include "core/VirtualTerminal.h"

using namespace toad::sim;

void test_terminal_lifecycle() {
    std::cout << "[Test 1] Testing VirtualTerminal lifecycle & PTY descriptors..." << std::endl;
    {
        VirtualTerminal vt("LifecycleTest", /*spawn_terminal=*/false, /*enable_input=*/false);
        assert(vt.is_open());
        assert(vt.master_fd() >= 0);
        assert(!vt.slave_path().empty());
        assert(vt.child_pid() > 0);
        std::cout << "  Slave device: " << vt.slave_path() << ", PID: " << vt.child_pid() << std::endl;
    }
    std::cout << "  -> PASSED (Clean RAII teardown)" << std::endl;
}

void test_terminal_write() {
    std::cout << "[Test 2] Testing VirtualTerminal write output..." << std::endl;
    VirtualTerminal vt("WriteTest", /*spawn_terminal=*/false, /*enable_input=*/false);

    // Open slave end to read what VirtualTerminal writes
    int sfd = open(vt.slave_path().c_str(), O_RDONLY);
    assert(sfd >= 0);

    std::string msg = "Hello VirtualTerminal!\n";
    ssize_t written = vt.write(msg);
    assert(written == static_cast<ssize_t>(msg.size()));

    char buf[128] = {0};
    ssize_t bytes_read = read(sfd, buf, sizeof(buf));
    assert(bytes_read > 0);
    std::string received(buf, bytes_read);
    assert(received.find("Hello VirtualTerminal!") != std::string::npos);
    close(sfd);

    std::cout << "  Slave read: " << received;
    std::cout << "  -> PASSED" << std::endl;
}

void test_terminal_bidirectional_input() {
    std::cout << "[Test 3] Testing VirtualTerminal bidirectional input callback..." << std::endl;
    std::atomic<bool> input_received{false};
    std::string captured_data;
    std::mutex data_mtx;

    VirtualTerminal vt("InputTest", /*spawn_terminal=*/false, /*enable_input=*/true,
        [&](const uint8_t* data, size_t size) {
            std::lock_guard<std::mutex> lock(data_mtx);
            captured_data.append(reinterpret_cast<const char*>(data), size);
            input_received.store(true);
        }
    );

    assert(vt.enable_input());

    // Write from slave device (simulating user typing in terminal)
    int sfd = open(vt.slave_path().c_str(), O_WRONLY);
    assert(sfd >= 0);
    write(sfd, "sensor_read\n", 12);
    close(sfd);

    // Wait for input callback
    for (int i = 0; i < 50 && !input_received.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(input_received.load());
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        assert(captured_data.find("sensor_read") != std::string::npos);
        std::cout << "  Captured callback input: " << captured_data;
    }
    std::cout << "  -> PASSED" << std::endl;
}

void test_terminal_input_toggle() {
    std::cout << "[Test 4] Testing VirtualTerminal set_enable_input toggle..." << std::endl;
    std::atomic<int> callback_count{0};

    VirtualTerminal vt("ToggleTest", /*spawn_terminal=*/false, /*enable_input=*/false,
        [&](const uint8_t*, size_t) {
            callback_count++;
        }
    );

    assert(!vt.enable_input());

    int sfd = open(vt.slave_path().c_str(), O_WRONLY);
    assert(sfd >= 0);
    write(sfd, "drop_this\n", 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    assert(callback_count.load() == 0);

    // Turn on input
    vt.set_enable_input(true);
    assert(vt.enable_input());

    write(sfd, "take_this\n", 10);
    close(sfd);

    for (int i = 0; i < 50 && callback_count.load() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(callback_count.load() >= 1);
    std::cout << "  Callback successfully fired after enable_input(true)." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    setenv("TOAD_SNOOPER_NO_GUI", "1", 1);
    std::cout << "=== Running VirtualTerminal Verification Tests ===" << std::endl;

    test_terminal_lifecycle();
    test_terminal_write();
    test_terminal_bidirectional_input();
    test_terminal_input_toggle();

    std::cout << "=== All VirtualTerminal Tests Passed Successfully! ===" << std::endl;
    return 0;
}

