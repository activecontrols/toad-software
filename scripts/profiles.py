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
