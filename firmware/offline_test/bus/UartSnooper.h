#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <memory>
#include <mutex>

#include "ITransactionObservable.h"

namespace toad::sim {

enum class SnoopFormat {
    FORMAT_ASCII,
    FORMAT_HEX,
    FORMAT_HEX_DUMP
};

class UartSnooper : public IUartObserver {
public:
    explicit UartSnooper(std::string bus_name = "UART",
                         SnoopFormat format = SnoopFormat::FORMAT_ASCII,
                         std::ostream* out_stream = &std::cout);

    ~UartSnooper() override = default;

    void set_format(SnoopFormat format) { format_ = format; }
    SnoopFormat format() const { return format_; }

    void set_output_stream(std::ostream* out_stream) { out_stream_ = out_stream; }
    void set_custom_sink(std::function<void(const std::string&)> sink) { custom_sink_ = std::move(sink); }
    void set_fd_sink(int fd) { fd_sink_ = fd; }

    void on_uart_transaction(const UartTransaction& tx) override;

    // Formatting utilities
    std::string format_transaction(const UartTransaction& tx) const;
    static std::string format_ascii(const UartTransaction& tx, const std::string& name);
    static std::string format_hex(const UartTransaction& tx, const std::string& name);
    static std::string format_hexdump(const UartTransaction& tx, const std::string& name);

private:
    std::string bus_name_;
    SnoopFormat format_{SnoopFormat::FORMAT_ASCII};
    std::ostream* out_stream_{nullptr};
    std::function<void(const std::string&)> custom_sink_{nullptr};
    int fd_sink_{-1};
    mutable std::mutex mtx_;
};

} // namespace toad::sim
