import os
import serial
import argparse
import sys
from profiles import select_profile
from config import get_serial_config


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Run a motor test profile.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose console output during telemetry loop")
    return parser.parse_args()

def main():
    args = parse_args()
    verbose = args.verbose
    profile_name, profile_func = select_profile()
    if verbose:
        print(f"Running profile: '{profile_name}' (verbose mode enabled)...")
    else:
        print(f"Running profile: '{profile_name}'...")

    ser = None
    fout = None

    try:
        # Ensure destination directory exists
        log_dir = "./dry_tests"
        os.makedirs(log_dir, exist_ok=True)

        # Load serial port configuration
        port, baudrate = get_serial_config()
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1) 

        # Dynamic output filename matching selected profile
        out_path = os.path.join(log_dir, f"out_{profile_name}.csv")
        fout = open(out_path, "w")

        ser.write(b"motor_follow_profile\n")

        # write the first line (the csv header)
        fout.write(ser.readline().decode("utf-8"))

        time_s = 0.0

        while True:
            do_quit, new_target = profile_func(time_s)

            if do_quit:
                break

            ser.write((f"g {new_target:.2f}\n").encode("utf-8"))

            # wait for data to come back
            data_response = ser.readline().decode("utf-8")

            # parse the time field
            time_s = float(data_response.split(',')[0])

            # pipe the csv data to output file
            fout.write(data_response)
            
            # Print to console if verbose
            if verbose:
                print(f"Time: {time_s:.4f}s | Target: {new_target:.2f}°")

    finally:
        # send quit command        
        if ser and ser.is_open:
            ser.write(b'q\n')
            ser.close()

        # close handles
        if fout and not fout.closed:
            fout.close()


if __name__ == "__main__":
    main()