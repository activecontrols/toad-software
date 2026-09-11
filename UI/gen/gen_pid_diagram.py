import sys

def get_location(key: str, item_location_db):
    if key.startswith('between'):
        a, b = key.removeprefix('between').split('and')
        x1, y1, _ = get_location(a, item_location_db)
        x2, y2, _ = get_location(b, item_location_db)
        if x1 == x2:
            loctype = 'hor'
        elif y1 == y2:
            loctype = 'vert'
        else:
            loctype = 'raw'
        return (x1+x2)/2, (y1+y2)/2, loctype

    if key.strip() in item_location_db:
        x, y, loctype = item_location_db[key.strip()]
    else:
        try:
            (x, y), loctype = key.split(' '), 'raw'
            x = float(x)
            y = float(y)
        except:
            raise Exception(f"Couldn't find key <{key.strip()}>")

    return x, y, loctype


def main():
    _, pid_fname, cpp_fname = sys.argv

    with open(pid_fname) as f:
        pid_lines = f.read()

    pid_lines, settings_lines = pid_lines.split("Settings:")
    pid_lines, pipe_lines = pid_lines.split("Pipes:")
    pid_lines, instrument_lines = pid_lines.split("Instruments:")
    valve_lines, item_lines = pid_lines.split("Other Items:")

    settings_lines = settings_lines.split('\n')
    pipe_lines = pipe_lines.strip().split('\n')[1:]
    instrument_lines = instrument_lines.strip().split('\n')[1:]
    item_lines = item_lines.strip().split('\n')[1:]
    valve_lines = valve_lines.strip().split('\n')[2:]

    pipe_lines = [line for line in pipe_lines if not line.startswith('//')]
    instrument_lines = [line for line in instrument_lines if not line.startswith('//')]
    item_lines = [line for line in item_lines if not line.startswith('//')]
    valve_lines = [line for line in valve_lines if not line.startswith('//')]

    item_location_db: dict[str, tuple[float, float, str]] = {}
    settings = {}

    valve_name_map = {}
    with open('firmware/lib/hardware_mapping/ec_valves.h') as f:
        for line in f.readlines():
            if (line.startswith('#define SV') or line.startswith('#define BV')) and '_default' not in line:
                name = line.split()[2]
                valve_name_map[name[:8]] = name

    sensor_name_map = {}
    with open('firmware/lib/hardware_mapping/ec_sensors.h') as f:
        for line in f.readlines():
            if (line.startswith('#define PT') or line.startswith('#define TC')):
                name = line.split()[2]
                sensor_name_map[name[:8]] = name


    for line in settings_lines:
        if line.strip():
            param, value = line.strip().split(" ")
            settings[param] = float(value)



    with open(cpp_fname, 'w+') as f:
        f.write("""
#include "pid_diagram.h"
#include "ec_sensors.h"
#include "ec_valves.h"
#include "flight_history.h"

PID_Diagram pid_diagram;

void init_diagram() {
                """.strip())
        f.write('\n')

        for line in valve_lines:
            if not line.strip():
                continue

            valve_type, name, x, y, hv, label_dir = (x.strip() for x in line.split(','))
            x, y = float(x), float(y)

            if label_dir == 'up':
                lx, ly = x, y - 30
            elif label_dir == 'down':
                lx, ly = x, y + 30
            elif label_dir == 'right':
                lx, ly = x + 60, y
            elif label_dir == 'left':
                lx, ly = x - 60, y
            else:
                raise Exception(f"Label direction not found {label_dir}")

            if valve_type != "Throttle":
                f.write(f"""  pid_diagram.valves[{valve_name_map[name.replace('-', '_')]}] = {{Valve_Type::{valve_type}, "{name}", {valve_name_map[name.replace('-', '_')] + '_default'}, ImVec2({x}, {y}), '{hv}', ImVec2({lx}, {ly})}};\n""")
            else:
                f.write(f"""  pid_diagram.throttle_valves[{1 if 'FU' in name else 0}] = {{"{name}", ImVec2({x}, {y}), '{hv}', ImVec2({lx}, {ly})}};\n""")

            if hv == 'V':
                item_location_db[name + " top"] = (x, y - settings['VALVE_SIZE'], 'vert')
                item_location_db[name + " bottom"] = (x, y + settings['VALVE_SIZE'], 'vert')
            elif hv == 'H':
                item_location_db[name + " left"] = (x - settings['VALVE_SIZE'], y, 'hor')
                item_location_db[name + " right"] = (x + settings['VALVE_SIZE'], y, 'hor')
            item_location_db[name] = (x, y, 'center')

        f.write('\n')

        item_counter = 0
        for line in item_lines:
            if not line.strip():
                continue

            item_type, name, x, y, orient = (x.strip() for x in line.split(','))
            item_type = item_type.replace(' ', '_')
            x, y = float(x), float(y)
            f.write(f"""  pid_diagram.pid_items[{item_counter}] = {{PID_Type::{item_type}, "{name}", ImVec2({x}, {y}), '{orient}'}};\n""")
            item_counter += 1
            
            if item_type == "Tank":
                item_location_db[name + " top"] = (x, y - settings['TANK_HEIGHT'] / 2 - settings['TANK_HEIGHT'] * 0.2, 'vert')
                item_location_db[name + " bottom"] = (x, y + settings['TANK_HEIGHT'] / 2 + settings['TANK_HEIGHT'] * 0.2, 'vert')
                item_location_db[name + " leftsensor"] = (x - 120, y, 'raw')
                item_location_db[name + " rightsensor"] = (x + 140, y, 'raw')
                item_location_db[name + " left"] = (x - settings['TANK_WIDTH'] / 2, y, 'hor')
                item_location_db[name + " right"] = (x + settings['TANK_WIDTH'] / 2, y, 'hor')
            if item_type == "Small_Nozzle":
                if orient == 'L':
                    item_location_db[name] = (x + settings['SMALL_NOZZEL_LENGTH'] / 2, y, 'hor')
                else:
                    item_location_db[name] = (x - settings['SMALL_NOZZEL_LENGTH'] / 2, y, 'hor')
            if item_type == "Large_Nozzle":
                item_location_db[name + " top"] = (x, y - settings['LARGE_NOZZEL_HEIGHT'] / 2, 'vert')
                item_location_db[name + " Ox Inj"] = (x - 130, y - settings['LARGE_NOZZEL_HEIGHT'] / 2 + 10, 'raw')
                item_location_db[name + " Fu Inj"] = (x - 130, y - settings['LARGE_NOZZEL_HEIGHT'] / 2 + 90, 'raw')
                item_location_db[name + " Ox Inj TC"] = (x - 250, y - settings['LARGE_NOZZEL_HEIGHT'] / 2 + 10, 'raw')
                item_location_db[name + " Fu Inj TC"] = (x - 250, y - settings['LARGE_NOZZEL_HEIGHT'] / 2 + 90, 'raw')
                item_location_db[name + " left"] = (x - settings['LARGE_NOZZEL_WIDTH'] / 2, y, 'raw')
                item_location_db[name + " right"] = (x - settings['LARGE_NOZZEL_WIDTH'] / 2, y, 'raw')
            if item_type == "Igniter":
                item_location_db[name + " topleft"] = (x - settings['SMALL_NOZZEL_WIDTH'] / 3, y - settings['SMALL_NOZZEL_LENGTH'] / 2, 'vert')
                item_location_db[name + " topright"] = (x + settings['SMALL_NOZZEL_WIDTH'] / 3, y - settings['SMALL_NOZZEL_LENGTH'] / 2, 'vert')
                item_location_db[name + " sensor"] = (x + 90, y - 10, 'raw')
                item_location_db[name + " left"] = (x - settings['SMALL_NOZZEL_WIDTH'] / 2, y - 10, 'raw')
                item_location_db[name + " right"] = (x + settings['SMALL_NOZZEL_WIDTH'] / 2, y - 10, 'raw')
            if item_type == 'Reg':
                item_location_db[name + " left"] = (x - settings['REG_SIZE'], y, 'hor')
                item_location_db[name + " right"] = (x + settings['REG_SIZE'], y, 'hor')
            if item_type == 'CheckValve':
                if orient == 'V':
                    item_location_db[name + " top"] = (x, y - settings['CHECK_VALVE_SIZE'], 'vert')
                    item_location_db[name + " bottom"] = (x, y + settings['CHECK_VALVE_SIZE'], 'vert')
                elif orient == 'H':
                    item_location_db[name + " left"] = (x - settings['CHECK_VALVE_SIZE'], y, 'hor')
                    item_location_db[name + " right"] = (x + settings['CHECK_VALVE_SIZE'], y, 'hor')
            if item_type == 'ManualValve':
                if orient == 'V':
                    item_location_db[name + " top"] = (x, y - settings['REG_SIZE'], 'vert')
                    item_location_db[name + " bottom"] = (x, y + settings['REG_SIZE'], 'vert')
                elif orient == 'H':
                    item_location_db[name + " left"] = (x - settings['REG_SIZE'], y, 'hor')
                    item_location_db[name + " right"] = (x + settings['REG_SIZE'], y, 'hor')
                
        f.write(f'  static_assert({item_counter} == NUMBER_OF_PID_ITEMS);\n\n')

        instrument_counter = 0
        for line in instrument_lines:
            if not line.strip():
                continue

            item_type, name, loc, attach, attach_dir = (x.strip() for x in line.split(','))
            item_type = item_type.replace(' ', '_')
            reading = f"&FlightHistory.{item_type.lower()}s.{sensor_name_map[name.replace('-', '_')]}"
            x, y, _ = get_location(loc, item_location_db)
            ax, ay, _ = get_location(attach, item_location_db)
            f.write(f"""  pid_diagram.instruments[{instrument_counter}] = {{Instrument_Type::{item_type}, "{name}", {reading}, ImVec2({x}, {y}), ImVec2({ax}, {ay}), '{attach_dir}'}};\n""")
            instrument_counter += 1

        f.write(f'  static_assert({instrument_counter} == NUMBER_OF_INSTRUMENTS);\n\n')

        pipe_counter = 0
        for line in pipe_lines:
            if not line.strip():
                continue

            pipe_control = line.split('|')
            line = pipe_control[0]
            start, end, color = (x.strip() for x in line.split(','))
            sx, sy, start_type = get_location(start, item_location_db)
            ex, ey, end_type = get_location(end, item_location_db)

            if len(pipe_control) > 1:
                fill_list = [valve_name_map[x.replace('-', '_')] for x in pipe_control[1].strip().split(" ")]
            else:
                fill_list = []
            fill_list = ', '.join(str(x) for x in fill_list)

            if len(pipe_control) > 2:
                purge_list = [valve_name_map[x.replace('-', '_')] for x in pipe_control[2].strip().split(" ")]
            else:
                purge_list = []
            purge_list = ', '.join(str(x) for x in purge_list)

            if sx != ex and sy != ey:
                # advanced routing
                if start_type == 'vert' and end_type in ['hor', 'raw', 'between']:
                    mx, my = sx, ey
                elif end_type == 'vert' and start_type in ['hor', 'raw', 'between']:
                    mx, my = ex, sy
                elif start_type == 'hor' and end_type in ['vert', 'raw', 'between']:
                    mx, my = ex, sy
                elif end_type == 'hor' and start_type in ['vert', 'raw', 'between']:
                    mx, my = sx, ey
                else:
                    mx, my = sx, sy
                    print("Smart pipe routing failed on line", line, start_type, end_type)

                f.write(f"""  pid_diagram.pipes[{pipe_counter}] = {{ImVec2({sx}, {sy}), ImVec2({ex}, {ey}), PID_COLOR_{color}, {{{fill_list}}}, {{{purge_list}}}, ImVec2({mx}, {my})}};\n""")
            
            else:
                f.write(f"""  pid_diagram.pipes[{pipe_counter}] = {{ImVec2({sx}, {sy}), ImVec2({ex}, {ey}), PID_COLOR_{color}, {{{fill_list}}}, {{{purge_list}}}}};\n""")
            pipe_counter += 1
        
        f.write(f'  static_assert({pipe_counter} == NUMBER_OF_PIPES);\n')
        f.write("}")

        print(f'#define NUMBER_OF_PID_ITEMS {item_counter}')
        print(f'#define NUMBER_OF_INSTRUMENTS {instrument_counter}')
        print(f'#define NUMBER_OF_PIPES {pipe_counter}')

        #print(item_location_db)

if __name__ == '__main__':
    main()