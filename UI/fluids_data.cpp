#include "fluids_data.h"
#include "pid_diagram.h"
#include <stdint.h>
#include <stdio.h>
#include <windows.h>

// Persist socket data
SOCKET sock = INVALID_SOCKET;
SOCKET cmd_sock = INVALID_SOCKET;
struct sockaddr_in cmd_dest_addr;

float sensor_readings[NUMBER_OF_INSTRUMENTS];
bool valve_states[NUMBER_OF_VALVES];
float fill_levels[3] = {1.0f, 0.9f, 0.9f};

#pragma pack(push, 1)
struct ec_telemetry_packet {
  uint8_t pt_crc;
  uint8_t padding[3];
  float pressures[12];
  float temperatures[6];
  uint32_t valve_state;
  float valve_angles[2];
  float fill_levels[3];
};

struct valve_command_packet {
  uint32_t commanded_valves;
  float ox_throttle;
  float fu_throttle;
};

struct legacy_telemetry_packet {
  float TK_N2_press;
  float TK_O2_press;
  float TK_FU_press;
  float O2_manifold_press;
  float FU_manifold_press;
  float chamber_press;

  bool BV_N2_02;
  bool BV_O2_03;
  bool BV_FU_03;
  bool BV_02_04;
  bool BV_FU_04;
  bool SV_N2_05;
  bool SV_N2_06;
  bool SV_N2_07;
  bool SV_N2_08;
};
#pragma pack(pop)

void commit_ec_packet(const ec_telemetry_packet &tp) {
  // Map pressures [psia]
  sensor_readings[PT_N2_01_IDX] = tp.pressures[0];
  sensor_readings[PT_O2_01_IDX] = tp.pressures[1];
  sensor_readings[PT_FU_01_IDX] = tp.pressures[2];
  sensor_readings[PT_N2_02_IDX] = tp.pressures[3];
  sensor_readings[PT_O2_02_IDX] = tp.pressures[4];
  sensor_readings[PT_FU_02_IDX] = tp.pressures[5];
  sensor_readings[PT_FU_04_IDX] = tp.pressures[6];
  sensor_readings[PT_N2_BULK_IDX] = tp.pressures[7];

  // Map temperatures [K]
  sensor_readings[TC_N2_01_IDX] = tp.temperatures[0];
  sensor_readings[TC_O2_01_IDX] = tp.temperatures[1];
  sensor_readings[TC_O2_02_IDX] = tp.temperatures[2];
  sensor_readings[TC_FU_01_IDX] = tp.temperatures[3];

  // Map fill levels (N2 COPV dependent on 4.5 ksi max pressure)
  float copv_p = sensor_readings[PT_N2_01_IDX];
  fill_levels[0] = copv_p > 4500.0f ? 1.0f : (copv_p < 0.0f ? 0.0f : copv_p / 4500.0f);
  fill_levels[1] = tp.fill_levels[1]; // LOX
  fill_levels[2] = tp.fill_levels[2]; // Fuel
}

void commit_legacy_packet(const legacy_telemetry_packet &tp) {
  sensor_readings[PT_FU_01_IDX] = tp.TK_FU_press;
  sensor_readings[PT_N2_01_IDX] = tp.TK_N2_press;
  sensor_readings[PT_O2_01_IDX] = tp.TK_O2_press;
  sensor_readings[PT_FU_02_IDX] = tp.FU_manifold_press;
  sensor_readings[PT_O2_02_IDX] = tp.O2_manifold_press;
  sensor_readings[PT_FU_04_IDX] = tp.chamber_press;

  valve_states[BV_N2_02_IDX] = tp.BV_N2_02;
  valve_states[BV_O2_03_IDX] = tp.BV_O2_03;
  valve_states[BV_FU_03_IDX] = tp.BV_FU_03;
  valve_states[BV_O2_04_IDX] = tp.BV_02_04;
  valve_states[BV_FU_04_IDX] = tp.BV_FU_04;
  valve_states[SV_N2_05_IDX] = tp.SV_N2_05;
  valve_states[SV_N2_06_IDX] = tp.SV_N2_06;
  valve_states[SV_N2_07_IDX] = tp.SV_N2_07;
}

void send_valve_command() {
  if (cmd_sock == INVALID_SOCKET) {
    return;
  }

  valve_command_packet cmd = {};

  // Build bitmask matching ec_valves.h:
  // Bits 0..3: SV-N2-01..04 (RCS)
  if (valve_states[SV_N2_01_IDX]) cmd.commanded_valves |= (1 << 0);
  if (valve_states[SV_N2_02_IDX]) cmd.commanded_valves |= (1 << 1);
  if (valve_states[SV_N2_03_IDX]) cmd.commanded_valves |= (1 << 2);
  if (valve_states[SV_N2_04_IDX]) cmd.commanded_valves |= (1 << 3);

  // Bits 4..6: SV-N2-05..07 (Purges)
  if (valve_states[SV_N2_05_IDX]) cmd.commanded_valves |= (1 << 4);
  if (valve_states[SV_N2_06_IDX]) cmd.commanded_valves |= (1 << 5);
  if (valve_states[SV_N2_07_IDX]) cmd.commanded_valves |= (1 << 6);

  // Bits 7..8: SV-O2-01, SV-FU-01 (DART Igniter)
  if (valve_states[SV_O2_01_IDX]) cmd.commanded_valves |= (1 << 7);
  if (valve_states[SV_FU_01_IDX]) cmd.commanded_valves |= (1 << 8);

  // Bits 9..10: BV-N2-01, BV-N2-02
  if (valve_states[BV_N2_01_IDX]) cmd.commanded_valves |= (1 << 9);
  if (valve_states[BV_N2_02_IDX]) cmd.commanded_valves |= (1 << 10);

  // Bits 11..13: BV-O2-01, BV-O2-02, BV-O2-03
  if (valve_states[BV_O2_01_IDX]) cmd.commanded_valves |= (1 << 11);
  if (valve_states[BV_O2_02_IDX]) cmd.commanded_valves |= (1 << 12);
  if (valve_states[BV_O2_03_IDX]) cmd.commanded_valves |= (1 << 13);

  // Bits 14..15: BV-FU-01, BV-FU-03
  if (valve_states[BV_FU_01_IDX]) cmd.commanded_valves |= (1 << 14);
  if (valve_states[BV_FU_03_IDX]) cmd.commanded_valves |= (1 << 15);

  // Bit 16: BV-N2-FILL
  if (valve_states[BV_N2_FILL_IDX]) cmd.commanded_valves |= (1 << 16);

  // Throttle positions (binary 0.0 or 1.0)
  cmd.ox_throttle = valve_states[BV_O2_04_IDX] ? 1.0f : 0.0f;
  cmd.fu_throttle = valve_states[BV_FU_04_IDX] ? 1.0f : 0.0f;

  sendto(cmd_sock, (const char *)&cmd, sizeof(cmd), 0, (struct sockaddr *)&cmd_dest_addr, sizeof(cmd_dest_addr));
}

void init_fluids_data() {
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);

  // Receiver Socket (Port 9000)
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock != INVALID_SOCKET) {
    u_long mode = 1; // nonblocking
    ioctlsocket(sock, FIONBIO, &mode);

    // Limit OS kernel receive buffer to 4 KB (~40 packets) to prevent stale packet accumulation
    int rcvbuf = 4096;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char *)&rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9000);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      printf("Error binding receiver socket to port 9000\n");
    }
  }

  // Transmitter Socket for Commands (Targeting 127.0.0.1:9001)
  cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&cmd_dest_addr, 0, sizeof(cmd_dest_addr));
  cmd_dest_addr.sin_family = AF_INET;
  cmd_dest_addr.sin_port = htons(9001);
  cmd_dest_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
}

void deinit_fluids_data() {
  if (sock != INVALID_SOCKET) {
    closesocket(sock);
    sock = INVALID_SOCKET;
  }
  if (cmd_sock != INVALID_SOCKET) {
    closesocket(cmd_sock);
    cmd_sock = INVALID_SOCKET;
  }
  WSACleanup();
}

void fluids_data_periodic() {
  if (sock == INVALID_SOCKET) return;

  struct sockaddr_in sender;
  uint8_t temp_buf[512];
  uint8_t latest_buf[512];
  int last_bytes = 0;

  // Non-blocking socket drain loop: drains all queued packets to commit the newest packet
  while (true) {
    int sender_len = sizeof(sender);
    int bytes = recvfrom(sock, (char *)temp_buf, sizeof(temp_buf), 0, (struct sockaddr *)&sender, &sender_len);
    if (bytes <= 0) {
      break;
    }
    memcpy(latest_buf, temp_buf, bytes);
    last_bytes = bytes;
  }

  if (last_bytes == sizeof(ec_telemetry_packet)) { // 100 bytes (EC_FMT)
    commit_ec_packet(*(const ec_telemetry_packet *)latest_buf);
  } else if (last_bytes == 288) { // 288 bytes (GNC 188 + EC 100)
    commit_ec_packet(*(const ec_telemetry_packet *)(latest_buf + 188));
  } else if (last_bytes == sizeof(legacy_telemetry_packet)) {
    commit_legacy_packet(*(const legacy_telemetry_packet *)latest_buf);
  }
}