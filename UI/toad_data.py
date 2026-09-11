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
# 2 floats (valve angles)
# 3 floats (fill levels)
EC_FMT = "1B" + "3x" + "12f" + "6f" + "2f" + "3f"

# < little-endian
PACKET_FMT = "<" + GNC_FMT + EC_FMT

def build_packet(row: dict) -> bytes:
    values = list(row.values())

    # most of this is just fake to generate some noise on the readings
    pts = [0.0] * 12
    # n2 tank
    pts[0] = (50 - float(row['elapsed_time'])) / 50 * 2000
    # o2/fu tank, n2 reg
    pts[1] = 500 + float(row['thrust_perc'])/70
    pts[3] = 500 + float(row['thrust_perc'])/68
    pts[7] = 500 + float(row['thrust_perc'])/66

    # injectors
    pts[4] = float(row['thrust_perc'])/100 * 300 + 10
    pts[8] = float(row['thrust_perc'])/100 * 300
    pts[10] = float(row['thrust_perc'])/100 * 290
    tcs = [270 + float(row['thrust_perc'])/20, 90 + float(row['thrust_perc'])/15, 
           2000 + float(row['thrust_perc']), 2000 + float(row['thrust_perc']) * 1.5, 0, 0]

    ox_angle = (float(row['thrust_perc'])-50)/50 * 50 + 20
    fu_angle = (float(row['thrust_perc'])-50)/50 * 50 + 22.3

    fill_levels = [
        (30 - float(row['elapsed_time'])) / 40,
        (30 - float(row['elapsed_time'])) / 40,
        (30 - float(row['elapsed_time'])) / 60 + 0.25,
    ]

    return struct.pack(PACKET_FMT, *(float(x) for x in values[0:15 + 19 + 4 + 3 + 1]), 
                       bool(float(row['GND_flag'])), bool(float(row['flight_armed'])), 
                       float(row['thrust_perc']), int(float(row['rtk_status'])),
                       float(row['gps_hor_prec']), float(row['gps_ver_prec']), int(float(row['gps_sat_count'])),
                       int(0), *pts, *tcs, ox_angle, fu_angle, *fill_levels)
 
 
def replay(csv_path, host, port, rate_hz):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
    try:
        for row in rows:
            packet = build_packet(row)        
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