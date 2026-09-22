#include "UartSnooper.h"
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <cctype>

namespace toad::sim {

UartSnooper::UartSnooper(std::string bus_name, SnoopFormat format, std::ostream* out_stream)
    : bus_name_(std::move(bus_name)), format_(format), out_stream_(out_stream) {}

std::string UartSnooper::format_ascii(const UartTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " " << to_string(tx.direction) << "] ";
    for (uint8_t b : tx.data) {
        if (b == '\r') {
            oss << "\\r";
        } else if (b == '\n') {
            oss << "\\n\n";
        } else if (b == '\t') {
            oss << "\\t";
        } else if (std::isprint(b)) {
            oss << static_cast<char>(b);
        } else {
            oss << ".";
        }
    }
    if (tx.data.empty() || tx.data.back() != '\n') {
        oss << "\n";
    }
    return oss.str();
}

std::string UartSnooper::format_hex(const UartTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " " << to_string(tx.direction)
        << "] (" << tx.data.size() << " bytes): ";
    for (size_t i = 0; i < tx.data.size(); ++i) {
        oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx.data[i]);
        if (i + 1 < tx.data.size()) {
            oss << " ";
        }
    }
    oss << "\n";
    return oss.str();
}

std::string UartSnooper::format_hexdump(const UartTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " " << to_string(tx.direction)
        << "] (" << tx.data.size() << " bytes):\n";

    const size_t bytes_per_line = 16;
    for (size_t offset = 0; offset < tx.data.size(); offset += bytes_per_line) {
        // Offset
        oss << "  " << std::hex << std::setw(4) << std::setfill('0') << offset << ": ";

        // Hex bytes
        size_t line_len = std::min(bytes_per_line, tx.data.size() - offset);
        for (size_t i = 0; i < bytes_per_line; ++i) {
            if (i < line_len) {
                oss << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(tx.data[offset + i]) << " ";
            } else {
                oss << "   ";
            }
            if (i == 7) oss << " ";
        }

        // ASCII sidebar
        oss << " |";
        for (size_t i = 0; i < line_len; ++i) {
            uint8_t b = tx.data[offset + i];
            oss << (std::isprint(b) ? static_cast<char>(b) : '.');
        }
        for (size_t i = line_len; i < bytes_per_line; ++i) {
            oss << " ";
        }
        oss << "|\n";
    }
    return oss.str();
}

std::string UartSnooper::format_transaction(const UartTransaction& tx) const {
    switch (format_) {
        case SnoopFormat::FORMAT_ASCII:
            return format_ascii(tx, bus_name_);
        case SnoopFormat::FORMAT_HEX:
            return format_hex(tx, bus_name_);
        case SnoopFormat::FORMAT_HEX_DUMP:
            return format_hexdump(tx, bus_name_);
        default:
            return format_ascii(tx, bus_name_);
    }
}

void UartSnooper::on_uart_transaction(const UartTransaction& tx) {
    std::string formatted = format_transaction(tx);

    std::lock_guard<std::mutex> lock(mtx_);
    if (custom_sink_) {
        custom_sink_(formatted);
    }
    if (out_stream_) {
        *out_stream_ << formatted << std::flush;
    }
    if (fd_sink_ >= 0) {
        ssize_t written = ::write(fd_sink_, formatted.data(), formatted.size());
        (void)written;
    }
}

} // namespace toad::sim
