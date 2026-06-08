# Command Protocol

Description of the thought process behind and the implementation of the communication protocol used on PSP's ASTRA and TOAD vehicles.

## Requirements

Encode human readable commands and compressed binary data.

Support convenience features like backspace.

A receiver should be able to connect to the data stream at any point and start processing valid packets.

Be as simple as reasonably possible.

## Design

Commands are sent as ascii characters. Commands can be followed by parameters, separated with a space (`set_gimbal_angle 5.0 -3.0`). Parameters can be ascii characters or binary data blobs. Commands are terminated by a newline, which works easily with exsting serial monitors, such as the platformio serial monitor or the VSCode serial monitor. Using a newline makes it easy to find when a new command starts. However, this makes encoding arbitrary binary data difficult, as that data could contain the newline character. In binary data blobs, all newlines are escaped with a backslash "escape character" `\`. Therefore, backslahes also have to be escaped, so sending `\-\n` results in `\\-\\n`. For operator convenience, backspace characters are parsed as little backspaces unless escaped. For maximum compatibility, `\r\n` is treated the same as `\n` by simply ignoring the carriage return.

## Implementation

See [firmware/lib/serial_comms/CommandRouter.cpp](../firmware/lib/serial_comms/CommandRouter.cpp).
UI function coming soon.