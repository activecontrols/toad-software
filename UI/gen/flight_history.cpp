
#include "flight_history.h"

flight_history_t FlightHistory;

int read_start_pos; // points to the oldest stored data
int read_end_pos; // points to the most recently written data, always equal to read_start + FLIGHT_HISTORY_LENGTH - 1
int write_pos; // points to the next location to write (same as read_start)
