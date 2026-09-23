#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

namespace toad::sim {

/**
 * @brief Cross-platform Virtual Terminal abstraction.
 *
 * Encapsulates pseudo-terminal (PTY) creation, GUI terminal spawning (e.g. xterm),
 * headless fallbacks, bidirectional input/output streaming, and child process lifecycle.
 *
 * All platform-dependent system headers (<pty.h>, <termios.h>, <sys/wait.h>, etc.)
 * are strictly isolated within the implementation file.
 */
class VirtualTerminal {
public:
    using InputCallback = std::function<void(const uint8_t* data, size_t size)>;

    explicit VirtualTerminal(std::string title = "Terminal",
                             bool spawn_terminal = true,
                             bool enable_input = false,
                             InputCallback on_input = nullptr);

    ~VirtualTerminal();

    VirtualTerminal(const VirtualTerminal&) = delete;
    VirtualTerminal& operator=(const VirtualTerminal&) = delete;

    // Terminal identification
    const std::string& title() const { return title_; }
    void set_title(const std::string& title) { title_ = title; }

    // State inspection
    bool is_open() const;
    bool enable_input() const { return enable_input_; }
    void set_enable_input(bool enable);
    void set_input_callback(InputCallback callback);

    // Platform-neutral descriptor inspection
    int master_fd() const { return pty_master_fd_; }
    int slave_fd() const { return pty_slave_fd_; }
    const std::string& slave_path() const { return pty_slave_path_; }
    int64_t child_pid() const { return child_pid_; }

    // Writing output to the terminal
    ssize_t write(const void* data, size_t size);
    ssize_t write(const std::string& str);
    void write_line(const std::string& line);

    // Explicit teardown
    void close();

private:
    void open_terminal(bool spawn_terminal);
    void start_input_thread();
    void stop_input_thread();

    std::string title_;
    bool enable_input_{false};
    InputCallback on_input_{nullptr};

    int pty_master_fd_{-1};
    int pty_slave_fd_{-1};
    std::string pty_slave_path_;
    int64_t child_pid_{-1};

    std::thread input_thread_;
    std::atomic<bool> input_running_{false};

    mutable std::mutex write_mtx_;
    mutable std::mutex callback_mtx_;
};

} // namespace toad::sim

