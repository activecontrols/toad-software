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

class UartBus;

enum class SnoopFormat {
    FORMAT_ASCII,
    FORMAT_HEX,
    FORMAT_HEX_DUMP
};

class UartSnooper : public IUartObserver {
public:
    explicit UartSnooper(std::string bus_name = "UART",
                         SnoopFormat format = SnoopFormat::FORMAT_ASCII,
                         std::ostream* out_stream = nullptr,
                         bool spawn_terminal = true,
                         bool enable_input = false,
                         std::shared_ptr<UartBus> bus = nullptr);

    ~UartSnooper() override = default;

    UartSnooper(const UartSnooper&) = delete;
    UartSnooper& operator=(const UartSnooper&) = delete;

    void set_format(SnoopFormat format) { format_ = format; }
    SnoopFormat format() const { return format_; }

    void set_output_stream(std::ostream* out_stream) { out_stream_ = out_stream; }
    void set_custom_sink(std::function<void(const std::string&)> sink) { custom_sink_ = std::move(sink); }
    void set_fd_sink(int fd) { fd_sink_ = fd; }

    // Input configuration and bus attachment
    void set_enable_input(bool enable);
    bool enable_input() const;

    void attach_bus(std::shared_ptr<UartBus> bus);
    std::shared_ptr<UartBus> bus() const;

    void on_uart_transaction(const UartTransaction& tx) override;

    // Formatting utilities
    std::string format_transaction(const UartTransaction& tx) const;
    static std::string format_ascii(const UartTransaction& tx, const std::string& name);
    static std::string format_hex(const UartTransaction& tx, const std::string& name);
    static std::string format_hexdump(const UartTransaction& tx, const std::string& name);

    // VirtualTerminal inspection and properties
    std::shared_ptr<VirtualTerminal> terminal() const { return terminal_; }
    int pty_master_fd() const { return terminal_ ? terminal_->master_fd() : -1; }
    const std::string& pty_slave_path() const { static const std::string empty; return terminal_ ? terminal_->slave_path() : empty; }
    int64_t child_pid() const { return terminal_ ? terminal_->child_pid() : -1; }

private:
    std::string bus_name_;
    SnoopFormat format_{SnoopFormat::FORMAT_ASCII};
    std::ostream* out_stream_{nullptr};
    std::function<void(const std::string&)> custom_sink_{nullptr};
    int fd_sink_{-1};

    std::shared_ptr<VirtualTerminal> terminal_;
    std::weak_ptr<UartBus> target_bus_;

    mutable std::mutex mtx_;
    mutable std::mutex bus_mtx_;
};

} // namespace toad::sim
