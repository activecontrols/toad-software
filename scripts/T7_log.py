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

    # Tracking overall session averages using Welford's algorithm (overflow-proof)
    sorted_channel_keys = sorted(CHANNELS.keys())
    channel_names = [f"{CHANNELS[ch][0]}_{CHANNELS[ch][1]}" for ch in sorted_channel_keys]
    total_samples = 0
    total_avg_voltages = [0.0] * len(channel_names)
    total_avg_pressures = [0.0] * len(channel_names)

    try:
        with open(out_path, "w") as fout:
            # Write CSV header (sorted by channel number)
            voltage_headers = [f"{name}_V" for name in channel_names]
            pressure_headers = [f"{name}_P" for name in channel_names]
            header = "Timestamp," + ",".join(voltage_headers) + "," + ",".join(pressure_headers) + "\n"
            fout.write(header)
            print(f"\nLogging to {out_path}")
            if verbose:
                print("Verbose mode enabled (1-second averages)")
            print("Press Ctrl+C to stop logging\n")

            start_time = time.time()
            last_log_time = start_time

            # Accumulators for 1-second rolling averages in verbose mode
            accumulated_voltages = [[] for _ in channel_names]
            accumulated_pressures = [[] for _ in channel_names]

            # --- DATA LOGGING LOOP ---
            try:
                while True:
                    elapsed = time.time() - start_time
                    voltage_dict, pressure_dict = read_pressures_and_voltages(lj_handle)
                    
                    # Extract values in channel order for CSV output
                    voltage_values = [voltage_dict[ch] for ch in sorted_channel_keys]
                    pressure_values = [pressure_dict[ch] for ch in sorted_channel_keys]
                    voltage_str = ",".join(f"{v:.6f}" for v in voltage_values)
                    pressure_str = ",".join(f"{p:.4f}" for p in pressure_values)
                    
                    # Log data row with timestamp
                    data_row = f"{elapsed:.4f},{voltage_str},{pressure_str}\n"
                    fout.write(data_row)

                    # Update total session running average (Welford's update: avg += (x - avg) / n)
                    total_samples += 1
                    for i in range(len(channel_names)):
                        total_avg_voltages[i] += (voltage_values[i] - total_avg_voltages[i]) / total_samples
                        total_avg_pressures[i] += (pressure_values[i] - total_avg_pressures[i]) / total_samples
                    
                    # Buffer samples for verbose averaging
                    if verbose:
                        for i in range(len(channel_names)):
                            accumulated_voltages[i].append(voltage_values[i])
                            accumulated_pressures[i].append(pressure_values[i])

                        # Emit average every 1 second
                        now = time.time()
                        if (now - last_log_time) >= 1.0:
                            num_samples = len(accumulated_voltages[0])
                            if num_samples > 0:
                                avg_voltages = [sum(v) / num_samples for v in accumulated_voltages]
                                avg_pressures = [sum(p) / num_samples for p in accumulated_pressures]

                                print(f"[{elapsed:8.4f}s | 1s-avg N={num_samples}]", end="")
                                for i in range(len(channel_names)):
                                    print(f"\t{channel_names[i]}:\tV={avg_voltages[i]:.6f}\tP={avg_pressures[i]:8.4f}\t|", end="")
                                print("")

                            # Reset buffers and timer
                            accumulated_voltages = [[] for _ in channel_names]
                            accumulated_pressures = [[] for _ in channel_names]
                            last_log_time = now
                    
                    time.sleep(0.01)  # ~100 Hz sampling rate

            except KeyboardInterrupt:
                print("\n\nStopping data logger...")

    finally:
        # Clean shutdown for hardware interfaces
        ljm.close(lj_handle)
        print(f"Log saved to {out_path}")

        # Display final total averages across the entire acquisition
        print("\n" + "=" * 70)
        print(f"SESSION TOTAL AVERAGES (Total Samples: {total_samples})")
        print("=" * 70)
        if total_samples > 0:
            for i, name in enumerate(channel_names):
                print(f"  {name:<15} -> Avg Voltage: {total_avg_voltages[i]:.6f} V  |  Avg Pressure: {total_avg_pressures[i]:8.4f}")
        else:
            print("  No samples recorded.")
        print("=" * 70 + "\n")


if __name__ == "__main__":
    main()