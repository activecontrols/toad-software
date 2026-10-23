import csv
import socket
import struct
import sys
import time
 
# 15 floats (z),
# 19 floats (est_x)
# 4 floats (output)
# 3 floats (target pos)
# 1 float (time)
# 2 bools (gnd, armed)
# 2 padding bytes
# 1 float (thrust)
# 1 int (rtk status)
# 2 floats (gps precision)
# 1 int (gps sat count)
GNC_FMT = "15f" + "19f" + "4f" + "3f" + "1f" + "2?" + "2x" + "1f" + "1i" + "2f" + "1i"

# 1 byte (PT CRC)
# 3 padding bytes
# 12 floats (PT readings)
# 6 floats (TC readings)
# 1 uint (valve state)
# 2 floats (valve angles)
# 3 floats (fill levels)
EC_FMT = "1B" + "3x" + "12f" + "6f" + "1I" + "2f" + "3f"

# < little-endian
PACKET_FMT = "<" + GNC_FMT + EC_FMT

last_rcs_fire_time = 0
rcs_hold = 0

def build_packet(row: dict) -> bytes:
    global last_rcs_fire_time
    global rcs_hold
    values = list(row.values())

    # most of this is just fake to generate some noise on the readings
    pts = [0.0] * 12
    # n2 tank
    pts[0] = (50 - row['elapsed_time']) / 50 * 2000
    # o2/fu tank, n2 reg
    pts[1] = 500 + row['thrust_perc']/70
    pts[3] = 500 + row['thrust_perc']/68
    pts[7] = 500 + row['thrust_perc']/66

    # injectors
    pts[4] = row['thrust_perc']/100 * 300 + 10
    pts[8] = row['thrust_perc']/100 * 300
    pts[10] = row['thrust_perc']/100 * 290
    tcs = [270 + row['thrust_perc']/20, 90 + row['thrust_perc']/15, 
           2000 + row['thrust_perc'], 2000 + row['thrust_perc'] * 1.5, 0, 0]

    ox_angle = (row['thrust_perc']-50)/50 * 50 + 20
    fu_angle = (row['thrust_perc']-50)/50 * 50 + 22.3

    valve_state = 0
    valve_state |= (1 << 10) # N2 press
    valve_state |=  (1 << 13) | (1 << 15) # run valves

    if row["elapsed_time"] < 0.6:
        valve_state |=  (1 << 7) | (1 << 8) # igniter
        ox_angle = 0
        fu_angle = 0

    if row['elapsed_time'] - last_rcs_fire_time > 1:
        last_rcs_fire_time = row["elapsed_time"]
        if row["roll_rad_sec_squared"] > 0.5:
            rcs_hold = 1
        elif row["roll_rad_sec_squared"] < 0:
            rcs_hold = -1
            
    if rcs_hold == 1 and row['elapsed_time'] - last_rcs_fire_time < 0.25:
        valve_state |=  (1 << 0) | (1 << 1) # rcs pos
    if rcs_hold == -1 and row['elapsed_time'] - last_rcs_fire_time < 0.25:
        valve_state |=  (1 << 2) | (1 << 3) # rcs neg

    
    fill_levels = [
        (30 - row['elapsed_time']) / 40,
        (30 - row['elapsed_time']) / 40,
        (30 - row['elapsed_time']) / 60 + 0.25,
    ]

    return struct.pack(PACKET_FMT, *(values[0:15 + 19 + 4 + 3 + 1]), 
                       bool(row['GND_flag']), bool(row['flight_armed']), 
                       row['thrust_perc'], int(row['rtk_status']),
                       row['gps_hor_prec'], row['gps_ver_prec'], int(row['gps_sat_count']),
                       int(0), *pts, *tcs, valve_state, ox_angle, fu_angle, *fill_levels)
 
 
def replay(csv_path, host, port, rate_hz):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
    try:
        for row in rows:
            packet = build_packet({k: float(v) for k, v in row.items()})        
            sock.sendto(packet, (host, port))
            time.sleep(1.0 / rate_hz)
 
            
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()
 
 
def main():
    replay("flight18.csv", "127.0.0.1", 9000, 1000)
 
 
if __name__ == "__main__":
    main()