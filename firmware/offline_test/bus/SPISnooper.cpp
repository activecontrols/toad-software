#include "SPISnooper.h"
#include <iomanip>
#include <sstream>
#include <cctype>
#include <unistd.h>

namespace toad::sim {

SPISnooper::SPISnooper(std::string bus_name,
                       SpiSnoopFormat format,
                       std::ostream* out_stream,
                       bool spawn_terminal,
                       bool enable_input,
                       VirtualTerminal::InputCallback on_input)
    : bus_name_(std::move(bus_name)), format_(format), out_stream_(out_stream) {
    terminal_ = std::make_shared<VirtualTerminal>(
        "SPI: " + bus_name_,
        spawn_terminal,
        enable_input,
        std::move(on_input)
    );
}

std::string SPISnooper::format_ascii(const SpiTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " CS:" << tx.cs_pin
        << " (" << (tx.device_name.empty() ? "None" : tx.device_name) << ")]\n";
    oss << "  MOSI (" << tx.mosi_data.size() << " B): ";
    for (uint8_t b : tx.mosi_data) {
        oss << (std::isprint(b) ? static_cast<char>(b) : '.');
    }
    oss << "\n  MISO (" << tx.miso_data.size() << " B): ";
    for (uint8_t b : tx.miso_data) {
        oss << (std::isprint(b) ? static_cast<char>(b) : '.');
    }
    oss << "\n";
    return oss.str();
}

std::string SPISnooper::format_hex(const SpiTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " CS:" << tx.cs_pin
        << " (" << (tx.device_name.empty() ? "None" : tx.device_name) << ")]\n";

    oss << "  MOSI (" << tx.mosi_data.size() << " B): ";
    for (size_t i = 0; i < tx.mosi_data.size(); ++i) {
        oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx.mosi_data[i]) << " ";
    }
    oss << "\n  MISO (" << tx.miso_data.size() << " B): ";
    for (size_t i = 0; i < tx.miso_data.size(); ++i) {
        oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx.miso_data[i]) << " ";
    }
    oss << "\n";
    return oss.str();
}

std::string SPISnooper::format_hexdump(const SpiTransaction& tx, const std::string& name) {
    std::ostringstream oss;
    oss << "[" << tx.timestamp_us << " us] [" << name << " CS:" << tx.cs_pin
        << " (" << (tx.device_name.empty() ? "None" : tx.device_name) << ")]\n";

    auto dump_bytes = [&](const std::string& label, const std::vector<uint8_t>& data) {
        oss << "  " << label << " (" << data.size() << " B):\n";
        const size_t bytes_per_line = 16;
        for (size_t offset = 0; offset < data.size(); offset += bytes_per_line) {
            oss << "    " << std::hex << std::setw(4) << std::setfill('0') << offset << ": ";
            size_t line_len = std::min(bytes_per_line, data.size() - offset);
            for (size_t i = 0; i < bytes_per_line; ++i) {
                if (i < line_len) {
                    oss << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(data[offset + i]) << " ";
                } else {
                    oss << "   ";
                }
                if (i == 7) oss << " ";
            }
            oss << " |";
            for (size_t i = 0; i < line_len; ++i) {
                uint8_t b = data[offset + i];
                oss << (std::isprint(b) ? static_cast<char>(b) : '.');
            }
            for (size_t i = line_len; i < bytes_per_line; ++i) {
                oss << " ";
            }
            oss << "|\n";
        }
    };

    dump_bytes("MOSI", tx.mosi_data);
    dump_bytes("MISO", tx.miso_data);
    return oss.str();
}

std::string SPISnooper::format_transaction(const SpiTransaction& tx) const {
    switch (format_) {
        case SpiSnoopFormat::FORMAT_ASCII:
            return format_ascii(tx, bus_name_);
        case SpiSnoopFormat::FORMAT_HEX:
            return format_hex(tx, bus_name_);
        case SpiSnoopFormat::FORMAT_HEX_DUMP:
            return format_hexdump(tx, bus_name_);
        default:
            return format_hex(tx, bus_name_);
    }
}

void SPISnooper::on_spi_transaction(const SpiTransaction& tx) {
    std::string formatted = format_transaction(tx);

    std::lock_guard<std::mutex> lock(mtx_);

    if (terminal_) {
        terminal_->write(formatted);
    }
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

