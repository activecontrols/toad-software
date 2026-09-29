import sys

#TODO - include comments

def parse_file(fname: str, structs: dict[str, dict[str, str]]):
    with open(fname) as f:
        telemetry_lines = f.readlines()
    
    active_struct: dict[str, str] = {}
    active_struct_name = ""

    for line in telemetry_lines:
        line = line.split('//')[0].strip().removesuffix(';')
        if line:
            if 'struct' in line:
                active_struct_name = line.removeprefix('struct').removesuffix('{').strip()
                continue

            if line == '}':
                structs[active_struct_name] = active_struct
                active_struct = {}
                active_struct_name = ""
                continue

            if active_struct_name:
                var_type, var_name = line.split()
                active_struct[var_name] = var_type


def gen_history_struct(type_name: str, obj_name: str, svars: dict[str, str], structs: dict[str, dict[str, str]], indent: str):
    if type_name:
        type_name = f" {type_name}"
    if obj_name:
        obj_name = f" {obj_name}"

    out = f"{indent}struct{type_name} {{\n"
    for var_name, var_type in svars.items():
        if var_type in structs:
            out += gen_history_struct("", var_name, structs[var_type], structs, indent + "  ")
        else:
            out += f"{indent}  {var_type} {var_name}[FLIGHT_HISTORY_LENGTH * 2];\n"
    out += f"{indent}}}{obj_name};\n"
    return out

def gen_expansion(prefix: str, var_type: str, structs: dict[str, dict[str, str]]):
    out = ""
    if var_type in structs:
        for var_name, new_type in structs[var_type].items():
            out += gen_expansion(f"{prefix}.{var_name}", new_type, structs)
    else:
        out += f"  FlightHistory{prefix}[write_pos] = packet{prefix};\n"
        out += f"  FlightHistory{prefix}[write_pos + FLIGHT_HISTORY_LENGTH] = packet{prefix};\n"
    return out


flight_data_h_start = """
#pragma once

#include "toad_telemetry.h"

// this is a ring buffer to create history graphs
// if packets arrive from earlier to later as ABCDE we store
// [{0 0 0 0} 0 0 0 0] - rs = 0, wp = 0/4
// [A {0 0 0 A} 0 0 0] - rs = 1, wp = 1/5
// [A B {0 0 A B} 0 0] - rs = 2, wp = 2/6
// [A B C {0 A B C} 0] - rs = 3, wp = 3/7
// [{A B C D} A B C D] - rs = 0, wp = 0/4
// [E {B C D E} B C D] - rs = 1, wp = 1/5

#define FLIGHT_HISTORY_LENGTH 1000\n
"""

flight_data_h_end = """
extern flight_history_t FlightHistory;

extern int read_start_pos;
extern int read_end_pos;
extern int write_pos;

#define fh_now(x) ((x)[read_end_pos])
#define fh_all(x) (&((x)[read_start_pos]))

void init_fh();
"""

flight_data_cpp_start = """
#include "flight_history.h"

flight_history_t FlightHistory;

int read_start_pos; // points to the oldest stored data
int read_end_pos; // points to the most recently written data, always equal to read_start + FLIGHT_HISTORY_LENGTH - 1
int write_pos; // points to the next location to write (same as read_start)

void init_fh() {
  write_pos = 0;
  read_start_pos = write_pos;
  read_end_pos = read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}
"""

flight_data_cpp_end = """
void update_fh_pos() {
  write_pos += 1;
  write_pos %= FLIGHT_HISTORY_LENGTH;
  read_start_pos = write_pos;
  read_end_pos = read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}
"""

def main():
    parse_fnames = sys.argv[1:]

    telemetry_structs = ['gnc_telemetry_t', 'ec_telemetry_t']
    output_struct = 'flight_history_t'
    output_h_fname = 'UI/gen/flight_history.h'
    output_cpp_fname = 'UI/gen/flight_history.cpp'

    structs: dict[str, dict[str, str]] = {}
    for fname in parse_fnames:
        parse_file(fname, structs)


    structs[output_struct] = {}
    for sname, svars in structs.items():
            if sname in telemetry_structs:
                structs[output_struct].update(svars)
        
    with open(output_h_fname, 'w+') as f:
        f.write(flight_data_h_start)
        f.write(gen_history_struct(output_struct, "", structs[output_struct], structs, ""))
        f.write(flight_data_h_end)
        for telem_struct in telemetry_structs:
            f.write(f"void commit_packet({telem_struct} packet);\n")
        f.write("void update_fh_pos();\n")

    with open(output_cpp_fname, 'w+') as f:
        f.write(flight_data_cpp_start)
        for telem_struct in telemetry_structs:
            f.write(f"\nvoid commit_packet({telem_struct} packet) {{\n")
            f.write(gen_expansion("", telem_struct, structs))
            f.write(f"}};\n")
        f.write(flight_data_cpp_end)

if __name__ == '__main__':
    main()