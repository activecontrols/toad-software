#pragma once
#include "stdlib.h"
#include <cstdint>

class Serial
{
private:
    bool _is_open = false;
    int tty_fd;
public:
    Serial();
    ~Serial();

    bool open(const char* fp);
    bool close(void);

    size_t write(const char* data, size_t len);
    size_t read(char* out, size_t len);

    inline size_t write(const void* data, size_t len)
    {
        return write(reinterpret_cast<const char*>(data), len);
    }

    inline size_t write(uint8_t data)
    {
        return write(reinterpret_cast<const char*>(&data), sizeof(data));
    }

    inline size_t read(void* data, size_t len)
    {
        return read(reinterpret_cast<char*>(data), len);
    }

    bool is_open();
};

// defined in main.cpp
extern Serial serial;