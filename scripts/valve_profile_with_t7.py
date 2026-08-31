import math
import os
import serial
import time
from labjack import ljm

# --- LABJACK CONFIGURATION ---
CHANNELS = {
    0: ("Valve Upstream", 11448836, 149.9736607, -4.64377129),  # NAME, SN, SLOPE, OFFSET
    2: ("Venturi Throat", 5589424, 99.28998837, 2.079860712),
    4: ("Venturi Upstream", 11409838, 147.9777035, -3.692041074)
}

# --- PROFILE DEFINITIONS ---
def step_response(time_s):
    max_time = 30.0
    if time_s > max_time:
        return (True, 0.0)
    steps = int(time_s // 3)
    angle = steps * 90
    return (False, angle % 360)

def profile_sawtooth(time_s):
    max_time = 30.0
    period = 5.0
    v_min, v_max = 0, 180
    if time_s > max_time:
        return (True, 0.0)
    phase = (time_s % period) / period
    angle = v_min + phase * (v_max - v_min)
    return (False, angle)

def profile_sine(time_s):
    max_time = 25.0
    period = 5.0
    amplitude = 90.0
    if time_s > max_time:
        return (True, 0.0)
    target_angle = 180.0 + amplitude * math.sin(2 * math.pi / period * time_s)
    if target_angle < 0:
        target_angle += 360.0
    return (False, target_angle)

last_time = 0
phase = 0

def profile_chirp(time_s):
    global phase, last_time
    t1 = 20
    maxFreq, minFreq = 3, 0.1
    if time_s > t1:
        return (True, 0.0)
    Freq = minFreq * (maxFreq / minFreq) ** (time_s / t1)
    phase = phase + Freq * (time_s - last_time)
    target_angle = 10 * math.cos(2 * math.pi * phase)
    last_time = time_s
    target_angle += 180.0
    if target_angle < 0:
        target_angle += 360.0
    return (False, target_angle)

# Profile Mapping Dictionary
PROFILES = {
    "1": ("Chirp Profile", profile_chirp),
    "2": ("Step Response", step_response),
    "3": ("Sawtooth Profile", profile_sawtooth),
    "4": ("Sine Wave Profile", profile_sine)
}

def select_profile():
    """Displays a CLI dialog to select the active profile."""
    print("\n--- Select Microcontroller Motion Profile ---")
    for key, (name, _) in PROFILES.items():
        print(f" [{key}] {name}")
    
    while True:
        choice = input("Enter option number (1-4): ").strip()
        if choice in PROFILES:
            profile_name, profile_func = PROFILES[choice]
            print(f"Selected: {profile_name}\n")
            return profile_name, profile_func
        print("Invalid selection. Please enter a valid number.")

def read_pressures(handle):
    """Reads raw voltages from LabJack AIN channels and calculates pressures."""
    ain_names = [f"AIN{c}" for c in CHANNELS.keys()]
    raw_voltages = ljm.eReadNames(handle, len(CHANNELS), ain_names)
    
    pressures = []
    for raw_v, (_, _, slope, offset) in zip(raw_voltages, CHANNELS.values()):
        pressures.append(raw_v * slope + offset)
    return pressures

def main():
    # Prompt for profile selection via interactive dialog
    profile_name, active_profile = select_profile()

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
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1)

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
            pressure_headers = [name for (name, _, _, _) in CHANNELS.values()]
            full_header = f"{base_header}," + ",".join(pressure_headers) + "\n"
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
                pressures = read_pressures(lj_handle)
                pressure_str = ",".join(f"{p:.4f}" for p in pressures)

                # Merge and log data row
                combined_row = f"{data_response},{pressure_str}\n"
                fout.write(combined_row)

            # Send quit signal to microcontroller
            ser.write(b'q\n')

    finally:
        # Clean shutdown for hardware interfaces
        ser.close()
        ljm.close(lj_handle)
        print(f"Session finished for {profile_name}. Log saved to {out_path}")

if __name__ == "__main__":
    main()