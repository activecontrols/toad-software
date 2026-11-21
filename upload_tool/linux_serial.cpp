#include "serial.h"

#include <fcntl.h>    // Contains file controls like O_RDWR, O_NOCTTY
#include <unistd.h>   // write(), read(), close()
#include <termios.h>  // Contains POSIX terminal control definitions
#include <cstring>    // strerror()
#include <cerrno>     // errno

Serial::Serial(){};

Serial::~Serial()
{
    close();
};

bool Serial::open(const char* fp)
{
    // 1. Open the TTY device
    // O_RDWR: Read/Write access
    // O_NOCTTY: Prevents this port from becoming the process's controlling terminal
    tty_fd = ::open(fp, O_RDWR | O_NOCTTY);

    if (tty_fd < 0) {
        return false;
    }

    // 2. Read existing settings into a termios struct
    struct termios tty;
    if (tcgetattr(tty_fd, &tty) != 0) {
        ::close(tty_fd);
        return false;
    }

    // 3. Configure parameters for "Raw" binary communication (8N1)
    tty.c_cflag &= ~PARENB;        // Clear parity bit (No parity)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field (1 stop bit)
    tty.c_cflag &= ~CSIZE;         // Clear current size mask
    tty.c_cflag |= CS8;            // Set 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (local connection)

    // Disable canonical mode (line-by-line processing), echoing, and signaling
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    // Disable software flow control and special handling of bytes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Prevent special output processing (newlines, map tabs, etc.)
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // 4. Set Read Timeout behavior (Blocking or Non-blocking)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5; // Timeout after 1 second of no data

    // 5. Set Baud Rate (9600 in this example)
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // 6. Apply configuration back to the port
    if (tcsetattr(tty_fd, TCSANOW, &tty) != 0) {
        ::close(tty_fd);
        return false;
    }

    _is_open = true;

    return true;
}

bool Serial::close(void)
{
    if (!is_open())
    {
        return false;
    }

    ::close(tty_fd);
    return true;
}

size_t Serial::write(const char* data, size_t len)
{
    if (!is_open()) return 0;

    size_t res = ::write(tty_fd, data, len);
    return res != (size_t)(-1) ? res : 0;
}

size_t Serial::read(char* out, size_t len)
{
    if (!is_open()) return false;
    size_t i = 0;

    while (i < len)
    {
        size_t receive_count = ::read(tty_fd, out + i, len - i);

        if ((receive_count < 1) || (receive_count == (size_t)(-1)))
        {
            // if we timed out or error
            break;
        }

        i += receive_count;
    }

    return i;
}

bool Serial::is_open()
{    
    return _is_open;
}