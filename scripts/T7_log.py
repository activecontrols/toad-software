"""T7 Data Logger - Continuously reads and logs pressure data from LabJack T7."""
import os
import time
import argparse
from labjack import ljm
from t7_calibration import CHANNELS, read_pressures_and_voltages


def main():
    """Main logging loop for T7 pressure data."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="T7 Data Logger")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose console output")
    args = parser.parse_args()
    verbose = args.verbose
    
    # Create logs directory if it doesn't exist
    log_dir = "./flow_test_logs"
    os.makedirs(log_dir, exist_ok=True)

    # LabJack Setup
    lj_handle = ljm.openS("ANY", "ANY", "ANY")
    print("Connected to LabJack T7")
    print("Configuring LabJack negative channels...")
    for channel in CHANNELS.keys():
        ljm.eWriteName(lj_handle, f"AIN{channel}_NEGATIVE_CH", channel + 1)

    log_filename = input("Enter output CSV log name (without extension): ").strip()
    if not log_filename:
        log_filename = f"t7_log_{int(time.time())}"
    
    out_path = os.path.join(log_dir, f"{log_filename}.csv")

    try:
        with open(out_path, "w") as fout:
            # Write CSV header (sorted by channel number)
            # Headers: Timestamp, RawVoltage_Ch0, RawVoltage_Ch2, RawVoltage_Ch4, Pressure_Ch0, Pressure_Ch2, Pressure_Ch4
            channel_names = [CHANNELS[ch][0] for ch in sorted(CHANNELS.keys())]
            voltage_headers = [f"{name}_V" for name in channel_names]
            pressure_headers = [f"{name}_P" for name in channel_names]
            header = "Timestamp," + ",".join(voltage_headers) + "," + ",".join(pressure_headers) + "\n"
            fout.write(header)
            print(f"\nLogging to {out_path}")
            if verbose:
                print("Verbose mode enabled")
            print("Press Ctrl+C to stop logging\n")

            start_time = time.time()

            # --- DATA LOGGING LOOP ---
            try:
                while True:
                    elapsed = time.time() - start_time
                    voltage_dict, pressure_dict = read_pressures_and_voltages(lj_handle)
                    # Extract values in channel order for CSV output
                    voltage_values = [voltage_dict[ch] for ch in sorted(voltage_dict.keys())]
                    pressure_values = [pressure_dict[ch] for ch in sorted(pressure_dict.keys())]
                    voltage_str = ",".join(f"{v:.6f}" for v in voltage_values)
                    pressure_str = ",".join(f"{p:.4f}" for p in pressure_values)
                    
                    # Log data row with timestamp
                    data_row = f"{elapsed:.4f},{voltage_str},{pressure_str}\n"
                    fout.write(data_row)
                    
                    # Print to console if verbose
                    if verbose:
                        print(f"[{elapsed:8.4f}s] {channel_names[0]}: V={voltage_values[0]:.6f} P={pressure_values[0]:8.4f} | "
                              f"{channel_names[1]}: V={voltage_values[1]:.6f} P={pressure_values[1]:8.4f} | "
                              f"{channel_names[2]}: V={voltage_values[2]:.6f} P={pressure_values[2]:8.4f}")
                    
                    time.sleep(0.01)  # ~100 Hz sampling rate

            except KeyboardInterrupt:
                print("\n\nStopping data logger...")

    finally:
        # Clean shutdown for hardware interfaces
        ljm.close(lj_handle)
        print(f"Log saved to {out_path}")


if __name__ == "__main__":
    main()
