#include "fluids_data.h"
#include "pid_diagram.h"
#include <stdio.h>
#include <windows.h>

// persist socket data
SOCKET sock;

float sensor_readings[NUMBER_OF_INSTRUMENTS];
bool valve_states[NUMBER_OF_VALVES];

struct telemetry_packet {
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

void commit_packet(telemetry_packet tp) {
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
  valve_states[SV_N2_08_IDX] = tp.SV_N2_08;
}

void init_fluids_data() {
  // setup socket
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  if (sock < 0) {
    printf("Error creating socket...");
    return;
  }

  // Make socket nonblocking
  u_long mode = 1;
  ioctlsocket(sock, FIONBIO, &mode);

  // Bind to localhost:9000
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));

  addr.sin_family = AF_INET;
  addr.sin_port = htons(9000);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  // use INADDR_ANY instead if desired

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    printf("Error binding socket...");
    return;
  }
}

void deinit_fluids_data() {
  closesocket(sock);
  WSACleanup();
}

void fluids_data_periodic() {
  struct sockaddr_in sender;
  int sender_len = sizeof(sender);

  telemetry_packet tp;
  int bytes = recvfrom(sock, (char *)&tp, sizeof(tp), 0, (struct sockaddr *)&sender, &sender_len);

  // MATLAB CODE
  // pt_vec = [X_cur(1), X_cur(3), X_cur(4), X_cur(11), X_cur(12), X_cur(13)] / 6895;
  // valve_vec = [ U(1), U(2), U(3), U(4), U(5), U(9), U(10), U(11), U(12) ] > 0;
  // pkt = [ typecast(single(pt_vec), "uint8"), uint8(valve_vec), uint8([ 0, 0, 0 ]) ];
  // write(u, pkt, "127.0.0.1", 9000);

  if (bytes == sizeof(tp)) {
    commit_packet(tp);
  } else if (bytes >= 0) {
    printf("rcv size error - update the matlab code: %d %d\n", bytes, sizeof(tp));
  }
}