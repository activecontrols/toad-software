#include "UartTerminalHarness.h"
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <vector>

namespace toad::sim {

UartTerminalHarness::UartTerminalHarness(std::shared_ptr<UartBus> bus, SnoopFormat format)
    : bus_(std::move(bus)) {
    snooper_ = std::make_shared<UartSnooper>(bus_->name(), format, &std::cout);
    bus_->add_observer(snooper_);
}

UartTerminalHarness::~UartTerminalHarness() {
    stop();
}

std::string UartTerminalHarness::enable_pty() {
    char name_buf[128] = {0};
    if (openpty(&pty_master_fd_, &pty_slave_fd_, name_buf, nullptr, nullptr) < 0) {
        return "";
    }
    pty_slave_path_ = name_buf;

    // Direct snooper output to the PTY master so external terminals see formatted traffic
    snooper_->set_fd_sink(pty_master_fd_);
    // Disable stdout output when PTY is active to keep host stdout clean
    snooper_->set_output_stream(nullptr);

    return pty_slave_path_;
}

void UartTerminalHarness::set_format(SnoopFormat format) {
    if (snooper_) {
        snooper_->set_format(format);
    }
}

void UartTerminalHarness::inject_input(const std::string& input) {
    if (bus_) {
        bus_->write_to_firmware(input);
    }
}

void UartTerminalHarness::inject_raw(const uint8_t* data, size_t size) {
    if (bus_) {
        bus_->write_to_firmware(data, size);
    }
}

void UartTerminalHarness::start_interactive() {
    if (running_.exchange(true)) return;
    reader_thread_ = std::thread(&UartTerminalHarness::reader_loop, this);
}

void UartTerminalHarness::stop() {
    if (!running_.exchange(false)) return;

    if (pty_master_fd_ >= 0) {
        close(pty_master_fd_);
        pty_master_fd_ = -1;
    }
    if (pty_slave_fd_ >= 0) {
        close(pty_slave_fd_);
        pty_slave_fd_ = -1;
    }

    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
}

void UartTerminalHarness::reader_loop() {
    int input_fd = (pty_master_fd_ >= 0) ? pty_master_fd_ : STDIN_FILENO;

    uint8_t buffer[256];
    while (running_.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(input_fd, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000; // 50 ms timeout for responsive loop exit

        int ret = select(input_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(input_fd, &read_fds)) {
            ssize_t bytes_read = ::read(input_fd, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                bus_->write_to_firmware(buffer, static_cast<size_t>(bytes_read));
            } else if (bytes_read == 0) {
                // EOF reached on input
                eof_reached_.store(true);
                break;
            } else if (bytes_read < 0 && errno != EAGAIN && errno != EINTR) {
                eof_reached_.store(true);
                break;
            }
        }
    }
}

} // namespace toad::sim
