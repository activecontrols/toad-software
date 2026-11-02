#!/usr/bin/env python3
"""Receive a NAND flash log dump over USB serial.

Protocol (must match logging.cpp):
  host -> "dump_logs\n"          trigger (CommandRouter)
  host -> 'c'                    request next page
  fw   -> 'c' + u32 page (LE) + page data + checksum_a + checksum_b
  fw   -> 'k'                    end of log (sent in reply to a 'c')
"""

import argparse
import csv
import os
import sys
import time
import serial

CONTINUE_CHAR = b"c"
KILL_CHAR = b"k"
DEFAULT_COMMAND = "dump_logs" 
DEFAULT_PAGE_SIZE = 2048


def read_exact(ser, size):
    """Read exactly size bytes or raise a timeout."""
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"Serial timeout: received {len(data)}/{size} bytes")
        data.extend(chunk)
    return bytes(data)


def calculate_checksum(data):
    """Match the firmware's two rolling 8-bit checksums."""
    a = 0
    b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def drain(ser, quiet_s=0.3, max_s=3.0):
    """Discard incoming bytes until the line has been quiet for quiet_s."""
    start = time.monotonic()
    last_rx = time.monotonic()
    while time.monotonic() - start < max_s:
        waiting = ser.in_waiting
        if waiting:
            ser.read(waiting)
            last_rx = time.monotonic()
        elif time.monotonic() - last_rx >= quiet_s:
            return
        else:
            time.sleep(0.02)


def dump_flash(port, baud, page_size, output, command, timeout, no_trigger):
    pages_received = 0
    bin_part = output + ".part"
    csv_path = output + ".csv"
    csv_part = csv_path + ".part"

    try:
        with serial.Serial(port=port, baudrate=baud, timeout=timeout) as ser:
            # Some boards reset when the port opens; give them time.
            time.sleep(1)
            ser.reset_input_buffer()

            if not no_trigger:
                ser.write(command.encode() + b"\n")
                ser.flush()
                drain(ser)

            print(f"Connected to {port}")
            print("Starting flash dump...")

            with open(bin_part, "wb") as binary, open(
                csv_part, "w", newline=""
            ) as index_file:
                writer = csv.writer(index_file)
                writer.writerow(["page_number", "file_offset"])

                while True:
                    ser.write(CONTINUE_CHAR)
                    ser.flush()

                    marker = read_exact(ser, 1)

                    if marker == KILL_CHAR:
                        print("Vehicle finished sending pages.")
                        break

                    if marker != CONTINUE_CHAR:
                        raise ValueError(f"Unexpected response: {marker.hex()}")

                    page_num = int.from_bytes(read_exact(ser, 4), "little")
                    page_data = read_exact(ser, page_size)
                    received = tuple(read_exact(ser, 2))
                    expected = calculate_checksum(page_data)

                    if received != expected:
                        raise ValueError(
                            f"Checksum mismatch on page {page_num}: "
                            f"received {received}, expected {expected}"
                        )

                    file_offset = binary.tell()
                    binary.write(page_data)
                    writer.writerow([page_num, file_offset])
                    pages_received += 1

                    if pages_received % 100 == 0:
                        print(f"Received {pages_received} pages")

    except (TimeoutError, ValueError, serial.SerialException) as exc:
        print(f"\nDump FAILED after {pages_received} pages: {exc}", file=sys.stderr)
        print(f"Partial data kept in {bin_part}", file=sys.stderr)
        return False

    os.replace(bin_part, output)
    os.replace(csv_part, csv_path)

    print("\nFlash dump complete!")
    print(f"Pages received: {pages_received}")
    print(f"Total bytes:    {pages_received * page_size}")
    print(f"Binary file:    {output}")
    print(f"Page index:     {csv_path}")
    if pages_received == 0:
        print("Note: 0 pages. Is the firmware calling Logging::write()?")
    return True


def main():
    p = argparse.ArgumentParser(description="Receive a NAND flash dump over USB serial.")
    p.add_argument("--port", required=True, help="Serial port (e.g. COM5 or /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--page-size", type=int, default=DEFAULT_PAGE_SIZE)
    p.add_argument("--output", default="flash_dump.bin")
    p.add_argument("--command", default=DEFAULT_COMMAND, help="CommandRouter name that starts the dump")
    p.add_argument("--timeout", type=float, default=5.0, help="Serial read timeout in seconds")
    p.add_argument("--no-trigger", action="store_true", help="Do not send the dump command")
    args = p.parse_args()

    ok = dump_flash(
        args.port,
        args.baud,
        args.page_size,
        args.output,
        args.command,
        args.timeout,
        args.no_trigger,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()