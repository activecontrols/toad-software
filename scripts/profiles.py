"""Profile definitions for valve testing."""
import math

# Global state for profile_chirp
last_time = 0
phase = 0


def step_response(time_s):
    """Step response - increments angle by 90 degrees every 3 seconds."""
    max_time = 30.0
    if time_s > max_time:
        return (True, 0.0)
    steps = int(time_s // 3)
    angle = steps * 90
    return (False, angle % 360)


def small_step_response(time_s):
    """small step response - increments angle by 10 degrees every 1 second."""
    max_time = 30.0
    if time_s > max_time:
        return (True, 0.0)
    steps = int(time_s // 1)
    angle = steps * 10
    return (False, angle % 360)


def profile_sawtooth(time_s):
    """Sawtooth wave profile."""
    max_time = 30.0
    period = 5.0
    v_min, v_max = 0, 180
    if time_s > max_time:
        return (True, 0.0)
    phase = (time_s % period) / period
    angle = v_min + phase * (v_max - v_min)
    return (False, angle)


def profile_sine(time_s):
    """Sine wave profile."""
    max_time = 25.0
    period = 5.0
    amplitude = 90.0
    if time_s > max_time:
        return (True, 0.0)
    target_angle = 180.0 + amplitude * math.sin(2 * math.pi / period * time_s)
    if target_angle < 0:
        target_angle += 360.0
    return (False, target_angle)


def profile_chirp(time_s):
    """Chirp profile with frequency sweep."""
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