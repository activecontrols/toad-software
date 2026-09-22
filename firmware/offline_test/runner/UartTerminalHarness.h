#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <thread>

#include "bus/UartBus.h"
#include "bus/UartSnooper.h"

namespace toad::sim {

class UartTerminalHarness {
public:
    explicit UartTerminalHarness(std::shared_ptr<UartBus> bus,
                                 SnoopFormat format = SnoopFormat::FORMAT_ASCII);
    ~UartTerminalHarness();

    // Enable POSIX pseudo-terminal (PTY)
    // Returns the slave device path (e.g. "/dev/pts/3") or empty string on failure.
    std::string enable_pty();

    bool has_pty() const { return pty_master_fd_ >= 0; }
    const std::string& pty_slave_path() const { return pty_slave_path_; }

    void set_format(SnoopFormat format);

    // Send input into the bus toward firmware
    void inject_input(const std::string& input);
    void inject_raw(const uint8_t* data, size_t size);

    // Start background reader thread (reading from stdin or PTY master)
    void start_interactive();

    // Stop harness
    void stop();

    bool is_running() const { return running_.load(); }
    bool eof_reached() const { return eof_reached_.load(); }

    std::shared_ptr<UartBus> bus() const { return bus_; }
    std::shared_ptr<UartSnooper> snooper() const { return snooper_; }

private:
    void reader_loop();

    std::shared_ptr<UartBus> bus_;
    std::shared_ptr<UartSnooper> snooper_;
    std::atomic<bool> running_{false};
    std::atomic<bool> eof_reached_{false};

    int pty_master_fd_{-1};
    int pty_slave_fd_{-1};
    std::string pty_slave_path_;

    std::thread reader_thread_;
};

} // namespace toad::sim
