#include "VirtualTerminal.h"

#include <iostream>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
// Windows ConPTY placeholder for future cross-platform Windows port
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
// POSIX / Linux Pseudo-Terminal implementation
#include <pty.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <sys/wait.h>
#include <sys/select.h>
#include <sys/types.h>
#endif

namespace toad::sim {

VirtualTerminal::VirtualTerminal(std::string title,
                                 bool spawn_terminal,
                                 bool enable_input,
                                 InputCallback on_input)
    : title_(std::move(title)), enable_input_(enable_input), on_input_(std::move(on_input)) {
    open_terminal(spawn_terminal);
}

VirtualTerminal::~VirtualTerminal() {
    close();
}

bool VirtualTerminal::is_open() const {
    return pty_master_fd_ >= 0;
}

void VirtualTerminal::set_enable_input(bool enable) {
    if (enable_input_ == enable) return;
    enable_input_ = enable;
    if (enable_input_) {
        start_input_thread();
    } else {
        stop_input_thread();
    }
}

void VirtualTerminal::set_input_callback(InputCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mtx_);
    on_input_ = std::move(callback);
}

#if !defined(_WIN32) && !defined(_WIN64)

void VirtualTerminal::open_terminal(bool spawn_terminal) {
    char slave_name[128] = {0};
    if (openpty(&pty_master_fd_, &pty_slave_fd_, slave_name, nullptr, nullptr) < 0) {
        perror("VirtualTerminal openpty");
        return;
    }
    pty_slave_path_ = slave_name;

    // Configure slave terminal attributes: raw mode with OPOST|ONLCR for clean newline translation
    struct termios tio;
    if (tcgetattr(pty_slave_fd_, &tio) == 0) {
        cfmakeraw(&tio);
        tio.c_oflag |= (OPOST | ONLCR);
        tcsetattr(pty_slave_fd_, TCSANOW, &tio);
    }

    pid_t pid = fork();
    if (pid < 0) {
        perror("VirtualTerminal fork");
        return;
    }

    if (pid == 0) {
        // --- CHILD PROCESS ---
        setpgid(0, 0); // Isolate child in its own process group

        if (pty_master_fd_ >= 0) {
            ::close(pty_master_fd_);
            pty_master_fd_ = -1;
        }

        const char* display = getenv("DISPLAY");
        const char* no_gui = getenv("TOAD_SNOOPER_NO_GUI");
        const char* in_ctest = getenv("CTEST_INTERACTIVE_DEBUG_MODE");

        bool can_spawn = spawn_terminal &&
                         (display != nullptr && *display != '\0') &&
                         (!no_gui || std::string(no_gui) != "1") &&
                         (!in_ctest);

        if (can_spawn) {
            std::string shell_cmd;
            if (enable_input_) {
                shell_cmd = "echo '=== " + title_ + " (" + slave_name + ") [Interactive] ==='; "
                            "trap 'kill 0' EXIT; "
                            "(cat " + slave_name + " 2>/dev/null & cat > " + slave_name + " 2>/dev/null; wait)";
            } else {
                shell_cmd = "echo '=== " + title_ + " (" + slave_name + ") ==='; cat " + slave_name;
            }

            // Launch xterm, fallback to x-terminal-emulator
            execlp("xterm", "xterm", "-title", title_.c_str(), "-e", "sh", "-c", shell_cmd.c_str(), (char*)nullptr);
            execlp("x-terminal-emulator", "x-terminal-emulator", "-T", title_.c_str(), "-e", "sh", "-c", shell_cmd.c_str(), (char*)nullptr);
        }

        // Headless / fallback output loop:
        int sfd = open(slave_name, O_RDONLY);
        if (sfd < 0) {
            sfd = pty_slave_fd_;
        }

        char buffer[1024];
        while (true) {
            ssize_t bytes_read = ::read(sfd, buffer, sizeof(buffer));
            if (bytes_read <= 0) {
                break;
            }
            ssize_t written = ::write(STDOUT_FILENO, buffer, static_cast<size_t>(bytes_read));
            (void)written;
        }

        if (sfd >= 0) {
            ::close(sfd);
        }
        _exit(0);
    }

    // --- PARENT PROCESS ---
    child_pid_ = pid;
    if (pty_slave_fd_ >= 0) {
        ::close(pty_slave_fd_);
        pty_slave_fd_ = -1;
    }

    std::cout << "[VirtualTerminal] " << title_ << " attached to PTY " << pty_slave_path_
              << " (PID " << child_pid_ << (enable_input_ ? ", Interactive" : "") << ")" << std::endl;

    if (enable_input_) {
        start_input_thread();
    }
}

ssize_t VirtualTerminal::write(const void* data, size_t size) {
    if (!data || size == 0 || pty_master_fd_ < 0) return 0;
    std::lock_guard<std::mutex> lock(write_mtx_);
    return ::write(pty_master_fd_, data, size);
}

ssize_t VirtualTerminal::write(const std::string& str) {
    return write(str.data(), str.size());
}

void VirtualTerminal::write_line(const std::string& line) {
    std::string s = line + "\n";
    write(s.data(), s.size());
}

void VirtualTerminal::start_input_thread() {
    if (input_running_.load() || pty_master_fd_ < 0) return;

    // Drain any stale buffered bytes in master PTY before starting reader thread
    char drain_buf[1024];
    while (true) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(pty_master_fd_, &rfds);
        struct timeval tv{0, 0};
        if (select(pty_master_fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0) {
            if (::read(pty_master_fd_, drain_buf, sizeof(drain_buf)) <= 0) break;
        } else {
            break;
        }
    }

    input_running_.store(true);
    input_thread_ = std::thread([this]() {
        char buffer[512];
        while (input_running_.load() && pty_master_fd_ >= 0) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(pty_master_fd_, &rfds);

            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50000; // 50ms select timeout for responsive termination

            int ret = select(pty_master_fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(pty_master_fd_, &rfds)) {
                ssize_t bytes_read = ::read(pty_master_fd_, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    InputCallback cb;
                    {
                        std::lock_guard<std::mutex> lock(callback_mtx_);
                        cb = on_input_;
                    }
                    if (cb) {
                        cb(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(bytes_read));
                    }
                } else if (bytes_read == 0) {
                    // EOF on master PTY
                    break;
                } else {
                    if (errno != EAGAIN && errno != EINTR) {
                        break;
                    }
                }
            }
        }
        input_running_.store(false);
    });
}

void VirtualTerminal::stop_input_thread() {
    input_running_.store(false);
    if (input_thread_.joinable()) {
        input_thread_.join();
    }
}

void VirtualTerminal::close() {
    stop_input_thread();

    if (pty_master_fd_ >= 0) {
        ::close(pty_master_fd_);
        pty_master_fd_ = -1;
    }

    if (child_pid_ > 0) {
        pid_t pid = static_cast<pid_t>(child_pid_);
        kill(-pid, SIGTERM);
        kill(pid, SIGTERM);
        for (int i = 0; i < 10; ++i) {
            int status = 0;
            pid_t res = waitpid(pid, &status, WNOHANG);
            if (res != 0) break;
            usleep(10000);
        }
        child_pid_ = -1;
    }
}

#else
// --- Windows ConPTY Port Stubs ---
void VirtualTerminal::open_terminal(bool spawn_terminal) {
    (void)spawn_terminal;
    std::cout << "[VirtualTerminal] Windows ConPTY backend stub for " << title_ << std::endl;
}

ssize_t VirtualTerminal::write(const void* data, size_t size) {
    (void)data; (void)size;
    return 0;
}

ssize_t VirtualTerminal::write(const std::string& str) {
    (void)str;
    return 0;
}

void VirtualTerminal::write_line(const std::string& line) {
    (void)line;
}

void VirtualTerminal::start_input_thread() {}
void VirtualTerminal::stop_input_thread() {}
void VirtualTerminal::close() {}

#endif

} // namespace toad::sim

