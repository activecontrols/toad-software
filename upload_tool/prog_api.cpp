#include <iostream>

#include "prog_api.h"
#include "util.h"
#include "can_bus_defs.h"
#include "serial.h"

using namespace CAN_Programmer;

/*
For Serial communications: messages are sent as follows:
raw_data[0] = len
raw_data[1] = cmd_id
raw_data[2] = data[0]
...
raw_data[len + 1] = data[len - 1]

(the len encodes the length of the data[], excluding the cmd_id)

All communications are initiated from the host side, and the G4 is expected to reply to every message.

*/

namespace Prog_API
{
uint8_t recv_buf[64];
uint8_t recv_len;
uint8_t recv_id;

constexpr const char TOOL_CMD[] = "tool_cmd\n";

bool receive_frame(void);

template <typename T>
inline T convert_frame(void)
{
    T res = {};

    memcpy(&res, recv_buf, recv_len);

    return res;
}


template <typename T>
bool send_command(T cmd = {})
{
    static constexpr size_t size = sizeof(T);
    static_assert(size < 0xFF);

    // Invoke tool_cmd handler through CommandRouter
    serial.write(TOOL_CMD, sizeof(TOOL_CMD) - 1);

    static constexpr uint8_t size_byte = size;

    // The tool_cmd will handle the custom frame format
    serial.write(&size_byte, sizeof(size_byte));

    serial.write(T::cmd_id);

    serial.write(&cmd, sizeof(T));

    return true;
}

void print_error(msg_error_t error_msg)
{
    // wasn't sure if there was a better way to do this
    // using temp buf to add null terminator in case the data is malformed and lacks one
    char tmp[sizeof(msg_error_t::msg) + 1];

    memcpy(tmp, error_msg.msg, sizeof(tmp));

    // Insert null-terminator for safety
    tmp[sizeof(tmp) - 1] = 0;

    std::cerr << tmp << std::endl;
    
    return;
}

bool enter_bootloader(void)
{
    ExitOnFail(send_command<msg_enter_bootloader_t>());

    ExitOnFail(receive_frame()); // wait for G4 to reply

    // only valid response to msg_enter_bootloader_t command is error
    ExitOnFail(recv_id == msg_error_t::cmd_id);

    msg_error_t err_msg = convert_frame<msg_error_t>();

    ExitOnFail(err_msg.code == msg_error_t::SUCCESS);

    return true;
}

bool enter_app(void)
{
    ExitOnFail(send_command<msg_reset_into_app_t>());

    ExitOnFail(receive_frame());

    // G4 is expected to respond with msg_error_t
    ExitOnFail(recv_id == msg_error_t::cmd_id);

    msg_error_t err = convert_frame<msg_error_t>();

    if (err.code != msg_error_t::SUCCESS)
    {
        print_error(err);

        return false;
    }

    return true;
}

bool spi_transfer(const uint8_t* mosi, uint8_t* miso, size_t len)
{
    size_t idx = 0;

    // Split mosi[] into valid frames that can be sent
    while (idx < len)
    {
        // construct message with up to the next n byte (however many can fit in a single frame)
        msg_spi_transfer_t cmd = {};

        size_t data_frame_len = (len - idx);

        // Clamp to max frame data length
        if (data_frame_len > sizeof(msg_spi_transfer_t::data))
        {
            data_frame_len = sizeof(msg_spi_transfer_t::data);
        }

        cmd.len = data_frame_len;
        memcpy(cmd.data, mosi + idx, data_frame_len);

        // transmit
        ExitOnFail(send_command(cmd));

        // wait for reply
        ExitOnFail(receive_frame());

        // G4 replies with msg_error_t if something went wrong
        if (recv_id == msg_error_t::cmd_id)
        {
            print_error(convert_frame<msg_error_t>());
            return false;
        }

        // On success, G4 replies with msg_spi_response_t
        // which contains the data that was seen on MISO

        ExitMsgOnFail(recv_id == msg_spi_response_t::cmd_id, "spi_transfer(): received invalid response cmd_id");

        msg_spi_response_t response = convert_frame<msg_spi_response_t>();

        ExitOnFail(response.len == data_frame_len);

        // copy MISO data to output buf
        if (miso != nullptr)
        {
            memcpy(miso + idx, response.data, response.len);
        }

        idx += data_frame_len;
    }

    return true;
}

bool receive_frame(void)
{
    static constexpr char TIMED_OUT_MSG[] = "receive_frame(): timed out";

    uint8_t tmp_size = 0;
    uint8_t tmp_id = 0;
    uint8_t tmp_data[sizeof(recv_buf)];

    size_t n_read = serial.read(&tmp_size, sizeof(tmp_size));

    ExitMsgOnFail(n_read == sizeof(tmp_size), TIMED_OUT_MSG);

    n_read = serial.read(&tmp_id, sizeof(tmp_id));

    ExitMsgOnFail(n_read == sizeof(tmp_id), TIMED_OUT_MSG);

    n_read = serial.read(tmp_data, tmp_size);

    ExitMsgOnFail(n_read == tmp_size, TIMED_OUT_MSG);

    // Copy data over
    recv_len = tmp_size;
    recv_id = tmp_id;
    memcpy(recv_buf, tmp_data, recv_len);
    return true;
}
} // namespace Prog_API