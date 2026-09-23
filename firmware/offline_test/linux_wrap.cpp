#include "linux_wrap.h"
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <stdio.h>

// Global structures to save and restore original terminal configurations
struct termios original_terminal_settings;

// Restores the terminal to its standard "line-by-line" mode upon exit
void restore_terminal_mode(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_settings);
}

// Configures terminal into non-canonical (instant) mode
void terminal_begin(void) {
    // Get current attributes
    tcgetattr(STDIN_FILENO, &original_terminal_settings);
    
    // Register the cleanup function to revert changes when program ends
    atexit(restore_terminal_mode);

    struct termios raw_mode = original_terminal_settings;
    
    // ~ICANON: Turns off line buffering (makes input instant)
    // ~ECHO: Prevents the terminal from printing back the typed character automatically
    raw_mode.c_lflag &= ~(ICANON | ECHO);
    
    // Apply changes immediately
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_mode);
}

// Non-blocking check to see if a character is ready to be read
bool input_waiting(void) {
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    // 0 timeout means select() polls and returns instantly
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0;
}

int read_char_noblock(void)
{
    if (input_waiting())
    {
        return getchar();
    }

    return -1;
}