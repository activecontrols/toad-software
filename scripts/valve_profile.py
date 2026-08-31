import os
import serial
import math

# step response - increments angle by 90 degrees every 5 seconds and wraps around
def step_response(time_s):
    max_time = 30.0

    # quit when max time is reached
    if time_s > max_time:
        return (True, 0.0)

    steps = int(time_s // 3)
    
    # Calculate total angle incremented (90 degrees per step)
    angle = steps * 90
    
    # Wrap the angle around so it stays within [0, 270] and returns to 0 at 360
    return (False, angle % 360)

def profile_sawtooth(time_s):
    max_time = 30.0
    period = 5.0
    v_min = 0
    v_max = 180

    if time_s > max_time:
        return (True, 0.0)

    phase = (time_s % period) / period  # Normalized time from 0.0 to 1.0
    angle =  v_min + phase * (v_max - v_min)

    return (False, angle)

def profile_sine(time_s):
    max_time = 25.0
    period = 5.0
    amplitude = 90.0

    if time_s > max_time:
        return (True, 0.0)

    target_angle = 180.0 + amplitude * math.sin(2 * math.pi / period * time_s)

    if target_angle < 0:
        target_angle += 360.0 # for better readability, make target angle always positive

    return (False, target_angle)

last_time = 0
phase = 0

def profile_chirp(time_s):
    global phase
    global last_time


    t1 = 20
    maxFreq = 3
    minFreq = 0.1
    Freq = minFreq * (maxFreq / minFreq) ** (time_s / t1)
    phase = phase + Freq * (time_s - last_time)
    target_angle = 10 * math.cos(2 * math.pi * phase)
    last_time = time_s

    if time_s > t1:
        return (True, 0.0)

    target_angle += 180.0
    if target_angle < 0:
        target_angle += 360.0 # for better readability, make target angle always positive

    return (False, target_angle)


# profile function returns a Tuple[bool,float] where the bool is false to signal that the profile is complete, and the float is the target angle
profile = profile_chirp


def main():
    # Ensure destination directory exists
    log_dir = "./dry_tests"
    os.makedirs(log_dir, exist_ok=True)

    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1) 

    out_path = os.path.join(log_dir, "out_chirp.csv")
    fout = open(out_path, "w")

    ser.write(b"motor_follow_profile\n")

    # write the first line (the csv header)
    fout.write(ser.readline().decode("utf-8"))

    time_s = 0.0

    while True:
        do_quit, new_target = profile(time_s)

        if do_quit:
            break

        ser.write((f"g {new_target:.2f}\n").encode("utf-8"))

        # wait for data to come back
        data_response = ser.readline().decode("utf-8")

        # parse the time field
        time_s = float(data_response.split(',')[0])

        # pipe the csv data to output file
        fout.write(data_response)

    # send quit command        
    ser.write(b'q\n')

    # close handles
    fout.close()
    ser.close()
    return


if __name__ == "__main__":
    main()