#include "flight_history.h"
#include <stdio.h>
#include <windows.h>

// persist socket data
SOCKET sock;

void init_flight_data() {
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

void deinit_flight_data() {
  closesocket(sock);
  WSACleanup();
}

void flight_data_periodic() {
  struct sockaddr_in sender;
  int sender_len = sizeof(sender);

  ec_telemetry_t tp;
  int bytes = recvfrom(sock, (char *)&tp, sizeof(tp), 0, (struct sockaddr *)&sender, &sender_len);

  // MATLAB CODE
  // pt_vec = [X_cur(1), X_cur(3), X_cur(4), X_cur(11), X_cur(12), X_cur(13)] / 6895;
  // valve_vec = [ U(1), U(2), U(3), U(4), U(5), U(9), U(10), U(11), U(12) ] > 0;
  // pkt = [ typecast(single(pt_vec), "uint8"), uint8(valve_vec), uint8([ 0, 0, 0 ]) ];
  // write(u, pkt, "127.0.0.1", 9000);

  if (bytes == sizeof(tp)) {
    commit_packet(tp);
    update_fh_pos();
  } else if (bytes >= 0) {
    printf("rcv size error - update the matlab code: %d %d\n", bytes, sizeof(tp));
  }
}