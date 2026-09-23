#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <memory>
#include <mutex>

#include "ITransactionObservable.h"
#include "core/VirtualTerminal.h"

namespace toad::sim {

enum class SpiSnoopFormat {
    FORMAT_ASCII,
    FORMAT_HEX,
    FORMAT_HEX_DUMP
};

class SPISnooper : public ISpiObserver {
public:
    explicit SPISnooper(std::string bus_name = "SPI",
                        SpiSnoopFormat format = SpiSnoopFormat::FORMAT_HEX,
                        std::ostream* out_stream = nullptr,
                        bool spawn_terminal = true,
                        bool enable_input = false,
                        VirtualTerminal::InputCallback on_input = nullptr);

    ~SPISnooper() override = default;

    SPISnooper(const SPISnooper&) = delete;
    SPISnooper& operator=(const SPISnooper&) = delete;

    void set_format(SpiSnoopFormat format) { format_ = format; }
    SpiSnoopFormat format() const { return format_; }

    void set_output_stream(std::ostream* out_stream) { out_stream_ = out_stream; }
    void set_custom_sink(std::function<void(const std::string&)> sink) { custom_sink_ = std::move(sink); }
    void set_fd_sink(int fd) { fd_sink_ = fd; }

    // Input configuration
    void set_enable_input(bool enable) { if (terminal_) terminal_->set_enable_input(enable); }
    bool enable_input() const { return terminal_ ? terminal_->enable_input() : false; }
    void set_input_callback(VirtualTerminal::InputCallback cb) { if (terminal_) terminal_->set_input_callback(std::move(cb)); }

    // Underlying terminal inspection
    std::shared_ptr<VirtualTerminal> terminal() const { return terminal_; }
    int pty_master_fd() const { return terminal_ ? terminal_->master_fd() : -1; }
    const std::string& pty_slave_path() const { static const std::string empty; return terminal_ ? terminal_->slave_path() : empty; }
    int64_t child_pid() const { return terminal_ ? terminal_->child_pid() : -1; }

    void on_spi_transaction(const SpiTransaction& tx) override;

    // Formatting utilities
    std::string format_transaction(const SpiTransaction& tx) const;
    static std::string format_ascii(const SpiTransaction& tx, const std::string& name);
    static std::string format_hex(const SpiTransaction& tx, const std::string& name);
    static std::string format_hexdump(const SpiTransaction& tx, const std::string& name);

private:
    std::string bus_name_;
    SpiSnoopFormat format_{SpiSnoopFormat::FORMAT_HEX};
    std::ostream* out_stream_{nullptr};
    std::function<void(const std::string&)> custom_sink_{nullptr};
    int fd_sink_{-1};

    std::shared_ptr<VirtualTerminal> terminal_;
    mutable std::mutex mtx_;
};

} // namespace toad::sim

