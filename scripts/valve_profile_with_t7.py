import os
import serial
import time
import argparse
from labjack import ljm
from profiles import step_response, profile_sawtooth, profile_sine, profile_chirp, small_step_response
from t7_calibration import CHANNELS, read_pressures_and_voltages
from config import get_serial_config


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Valve Profile with T7 Data Logging")
    parser.add_argument(
        "-p", "--profile",
        choices=["chirp", "step", "sawtooth", "sine"],
        help="Profile to run"
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose console output during telemetry loop")
    return parser.parse_args()


# Profile Mapping Dictionary
PROFILES = {
    "chirp": ("Chirp Profile", profile_chirp),
    "step": ("Step Response", step_response),
    "sawtooth": ("Sawtooth Profile", profile_sawtooth),
    "sine": ("Sine Wave Profile", profile_sine),
    "small_step": ("Small Step Response", small_step_response)
}

def select_profile(profile_key: str = None):
    """Displays a CLI dialog to select the active profile or uses provided profile key."""
    if profile_key is None:
        print("\n--- Select Microcontroller Motion Profile ---")
        keys = list(PROFILES.keys())
        for idx, key in enumerate(keys, 1):
            name, _ = PROFILES[key]
            print(f" [{idx}] {name}")
        
        while True:
            choice = input(f"Enter option number (1-{len(keys)}) or profile name: ").strip().lower()
            if choice in PROFILES:
                profile_key = choice
                break
            elif choice.isdigit() and 1 <= int(choice) <= len(keys):
                profile_key = keys[int(choice) - 1]
                break
            print("Invalid selection. Please enter a valid number or profile name.")
    
    if profile_key not in PROFILES:
        print(f"Error: Unknown profile '{profile_key}'")
        return None, None
    
    profile_name, profile_func = PROFILES[profile_key]
    print(f"Selected: {profile_name}\n")
    return profile_name, profile_func

def main():
    # Parse command-line arguments
    args = parse_args()
    verbose = args.verbose
    
    # Prompt for profile selection via interactive dialog or use provided argument
    profile_name, active_profile = select_profile(args.profile)
    if active_profile is None:
        return
    
    if verbose:
        print("Verbose mode enabled")

    # Create logs directory if it doesn't exist
    log_dir = "./flow_test_logs"
    os.makedirs(log_dir, exist_ok=True)

    # --- HARDWARE SETUP ---
    # LabJack Setup
    lj_handle = ljm.openS("ANY", "ANY", "ANY")
    print("Configuring LabJack negative channels...")
    for channel in CHANNELS.keys():
        ljm.eWriteName(lj_handle, f"AIN{channel}_NEGATIVE_CH", channel + 1)

    # Serial Setup
    port, baudrate = get_serial_config()
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    log_filename = input("Enter output CSV log name (without extension): ").strip()
    if not log_filename:
        log_filename = f"log_{int(time.time())}"
    
    out_path = os.path.join(log_dir, f"{log_filename}.csv")

    try:
        with open(out_path, "w") as fout:
            # Start microcontroller profile execution
            ser.write(b"motor_follow_profile\n")

            # Read original header from microcontroller and append LabJack columns
            base_header = ser.readline().decode("utf-8").strip()
            channel_names = [CHANNELS[ch][0] for ch in sorted(CHANNELS.keys())]
            voltage_headers = [f"{name}_V" for name in channel_names]
            pressure_headers = [f"{name}_P" for name in channel_names]
            full_header = f"{base_header}," + ",".join(voltage_headers) + "," + ",".join(pressure_headers) + "\n"
            fout.write(full_header)

            time_s = 0.0

            # --- CONTROL & TELEMETRY LOOP ---
            while True:
                do_quit, new_target = active_profile(time_s)

                if do_quit:
                    break

                # Send command to microcontroller
                ser.write((f"g {new_target:.2f}\n").encode("utf-8"))

                # Microcontroller drives timing: blocking read waits for data line
                data_response = ser.readline().decode("utf-8").strip()

                if not data_response:
                    continue  # Safety check for serial timeout

                # Parse timestamp from the first CSV column sent by microcontroller
                time_s = float(data_response.split(',')[0])

                # Synchronously read and calculate pressures from LabJack
                voltage_dict, pressure_dict = read_pressures_and_voltages(lj_handle)
                # Extract values in channel order for CSV output
                voltage_values = [voltage_dict[ch] for ch in sorted(voltage_dict.keys())]
                pressure_values = [pressure_dict[ch] for ch in sorted(pressure_dict.keys())]
                voltage_str = ",".join(f"{v:.6f}" for v in voltage_values)
                pressure_str = ",".join(f"{p:.4f}" for p in pressure_values)

                # Merge and log data row
                combined_row = f"{data_response},{voltage_str},{pressure_str}\n"
                fout.write(combined_row)
                
                # Print to console if verbose
                if verbose:
                    print(f"Time: {time_s:.4f}s | Target: {new_target:.2f}° | Voltages: {voltage_values[0]:.6f}, {voltage_values[1]:.6f}, {voltage_values[2]:.6f} | Pressures: {pressure_values[0]:.4f}, {pressure_values[1]:.4f}, {pressure_values[2]:.4f}")

            # Send quit signal to microcontroller
            ser.write(b'q\n')

    finally:
        # Clean shutdown for hardware interfaces
        ser.close()
        ljm.close(lj_handle)
        print(f"Session finished for {profile_name}. Log saved to {out_path}")

if __name__ == "__main__":
    main()