import os
import serial
import argparse
import sys
from profiles import step_response, profile_sawtooth, profile_sine, profile_chirp


# Dictionary mapping profile keys to their functions
PROFILES = {
    "step": step_response,
    "sawtooth": profile_sawtooth,
    "sine": profile_sine,
    "chirp": profile_chirp,
}

def select_profile() -> tuple[str, callable]:
    """Handles profile selection via command-line flags or interactive terminal prompt."""
    parser = argparse.ArgumentParser(description="Run a motor test profile.")
    parser.add_argument(
        "-p", "--profile",
        choices=list(PROFILES.keys()),
        help="Profile to run"
    )
    args = parser.parse_args()

    profile_key = args.profile

    # Prompt interactively if not provided via CLI
    if not profile_key:
        print("\nAvailable profiles:")
        keys = list(PROFILES.keys())
        for idx, name in enumerate(keys, 1):
            print(f"  {idx}. {name}")
        
        while True:
            choice = input(f"\nSelect profile [1-{len(keys)} or name]: ").strip().lower()
            if choice in PROFILES:
                profile_key = choice
                break
            elif choice.isdigit() and 1 <= int(choice) <= len(keys):
                profile_key = keys[int(choice) - 1]
                break
            print("Invalid selection. Please try again.")

    return profile_key, PROFILES[profile_key]


def main():
    profile_name, profile_func = select_profile()
    print(f"Running profile: '{profile_name}'...")

    ser = None
    fout = None

    try:
        # Ensure destination directory exists
        log_dir = "./dry_tests"
        os.makedirs(log_dir, exist_ok=True)

        ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1) 

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