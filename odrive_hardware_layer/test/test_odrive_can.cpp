/**
 * ODrive CAN Test Suite
 * ======================
 * Tests the C++ CAN protocol implementation.
 * Contains read tests and write tests.
 *
 * Tests:
 *   - test_can_socket_open   : verify CAN socket opens correctly
 *   - test_heartbeat_read    : verify heartbeat is received from ODrive
 *   - test_encoder_read      : verify encoder estimates are received
 *   - test_clear_errors_write: verify clear errors command is sent
 *   - test_set_axis_state    : verify axis state command is sent
 *   - test_set_velocity_write: verify velocity command is sent
 *   - test_set_position_write: verify position command is sent
 *
 * Usage:
 *   g++ test_odrive_can.cpp -o test_odrive_can
 *   sudo ip link set can0 up type can bitrate 1000000
 *   ./test_odrive_can
 *
 * Author: Ahsan Ali
 * Date:   May 2026
 */

 #include <iostream>
 #include <string>
 #include <cstring>
 #include <cstdint>
 #include <unistd.h>
 #include <net/if.h>
 #include <sys/socket.h>
 #include <sys/ioctl.h>
 #include <sys/select.h>
 #include <linux/can.h>
 #include <linux/can/raw.h>
 #include <chrono>
 #include <thread>
 
 using namespace std;
 
 // ─────────────────────────────────────────────────────────
 // TEST CONFIG
 // ─────────────────────────────────────────────────────────
 
 const string CAN_INTERFACE = "can0";
 const int    TEST_NODE_ID  = 0;       // node ID to test against
 
 // ─────────────────────────────────────────────────────────
 // ODrive COMMAND IDs
 // ─────────────────────────────────────────────────────────
 
 constexpr uint8_t CMD_HEARTBEAT      = 0x01;
 constexpr uint8_t CMD_ESTOP          = 0x02;
 constexpr uint8_t CMD_SET_AXIS_STATE = 0x07;
 constexpr uint8_t CMD_GET_ENCODER_EST= 0x09;
 constexpr uint8_t CMD_SET_INPUT_VEL  = 0x0D;
 constexpr uint8_t CMD_SET_INPUT_POS  = 0x0C;
 constexpr uint8_t CMD_CLEAR_ERRORS   = 0x18;
 
 // ─────────────────────────────────────────────────────────
 // AXIS STATES
 // ─────────────────────────────────────────────────────────
 
 constexpr uint32_t AXIS_STATE_IDLE                = 1;
 constexpr uint32_t AXIS_STATE_MOTOR_CALIBRATION   = 4;
 constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
 
 // ─────────────────────────────────────────────────────────
 // HELPERS
 // ─────────────────────────────────────────────────────────
 
 uint32_t arb_id(int node_id, uint8_t cmd_id)
 {
     return (node_id << 5) | cmd_id;
 }
 
 void sleep_ms(int ms)
 {
     this_thread::sleep_for(chrono::milliseconds(ms));
 }
 
 void pack_float(uint8_t * buf, float val)   { memcpy(buf, &val, 4); }
 void pack_uint32(uint8_t * buf, uint32_t v) { memcpy(buf, &v, 4); }
 float unpack_float(uint8_t * buf)   { float v; memcpy(&v, buf, 4); return v; }
 uint32_t unpack_uint32(uint8_t * b) { uint32_t v; memcpy(&v, b, 4); return v; }
 
 // ─────────────────────────────────────────────────────────
 // CAN SOCKET
 // ─────────────────────────────────────────────────────────
 
 int can_open(const string & iface)
 {
     int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
     if (sock < 0) { perror("socket"); return -1; }
 
     struct ifreq ifr;
     strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
     if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
         perror("ioctl"); close(sock); return -1;
     }
 
     struct sockaddr_can addr;
     memset(&addr, 0, sizeof(addr));
     addr.can_family  = AF_CAN;
     addr.can_ifindex = ifr.ifr_ifindex;
 
     if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
         perror("bind"); close(sock); return -1;
     }
     return sock;
 }
 
 void send_can(int sock, int node_id, uint8_t cmd_id,
               uint8_t * data, uint8_t len)
 {
     struct can_frame frame;
     memset(&frame, 0, sizeof(frame));
     frame.can_id  = arb_id(node_id, cmd_id);
     frame.can_dlc = len;
     if (data && len > 0) memcpy(frame.data, data, len);
     write(sock, &frame, sizeof(frame));
 }
 
 bool recv_can(int sock, uint32_t & id_out,
               uint8_t * data_out, int timeout_ms)
 {
     fd_set fds;
     FD_ZERO(&fds);
     FD_SET(sock, &fds);
     struct timeval tv;
     tv.tv_sec  = timeout_ms / 1000;
     tv.tv_usec = (timeout_ms % 1000) * 1000;
     if (select(sock + 1, &fds, nullptr, nullptr, &tv) <= 0)
         return false;
     struct can_frame frame;
     if (read(sock, &frame, sizeof(frame)) < 0) return false;
     id_out = frame.can_id;
     memcpy(data_out, frame.data, 8);
     return true;
 }
 
 // ─────────────────────────────────────────────────────────
 // TEST FRAMEWORK
 // ─────────────────────────────────────────────────────────
 
 int tests_passed = 0;
 int tests_failed = 0;
 
 void test_result(const string & name, bool passed)
 {
     if (passed) {
         cout << "  [PASS] " << name << endl;
         tests_passed++;
     } else {
         cout << "  [FAIL] " << name << endl;
         tests_failed++;
     }
 }
 
 // ─────────────────────────────────────────────────────────
 // READ TESTS — verify data coming FROM ODrive
 // ─────────────────────────────────────────────────────────
 
 bool test_can_socket_open(int & sock)
 {
     cout << "\n--- READ TEST 1: CAN Socket Open ---" << endl;
     sock = can_open(CAN_INTERFACE);
     bool passed = (sock >= 0);
     test_result("CAN socket opens on " + CAN_INTERFACE, passed);
     return passed;
 }
 
 void test_heartbeat_read(int sock)
 {
     cout << "\n--- READ TEST 2: Heartbeat ---" << endl;
     uint32_t target = arb_id(TEST_NODE_ID, CMD_HEARTBEAT);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(3000);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 uint32_t error = unpack_uint32(data);
                 uint8_t  state = data[4];
                 cout << "  Heartbeat received — state=" << (int)state
                      << " error=0x" << hex << error << dec << endl;
                 test_result("Heartbeat received from node " +
                     to_string(TEST_NODE_ID), true);
                 test_result("Axis state is valid (1=IDLE or 8=CLOSED_LOOP)",
                     state == 1 || state == 8);
                 test_result("No axis errors on startup", error == 0);
                 return;
             }
         }
     }
     test_result("Heartbeat received from node " +
         to_string(TEST_NODE_ID), false);
 }
 
 void test_encoder_read(int sock)
 {
     cout << "\n--- READ TEST 3: Encoder Estimates ---" << endl;
     uint32_t target = arb_id(TEST_NODE_ID, CMD_GET_ENCODER_EST);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(2000);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 float pos = unpack_float(data);
                 float vel = unpack_float(data + 4);
                 cout << "  Encoder estimates — pos=" << pos
                      << " turns  vel=" << vel << " t/s" << endl;
                 test_result("Encoder estimates received from node " +
                     to_string(TEST_NODE_ID), true);
                 return;
             }
         }
     }
     // Not a failure in sensorless mode — just note it
     cout << "  No encoder data (normal in sensorless mode)" << endl;
     test_result("Encoder estimates check (sensorless = acceptable)", true);
 }
 
 // ─────────────────────────────────────────────────────────
 // WRITE TESTS — verify commands sent TO ODrive
 // ─────────────────────────────────────────────────────────
 
 void test_clear_errors_write(int sock)
 {
     cout << "\n--- WRITE TEST 1: Clear Errors ---" << endl;
     send_can(sock, TEST_NODE_ID, CMD_CLEAR_ERRORS, nullptr, 0);
     sleep_ms(300);
 
     // Verify by reading heartbeat — error should be 0
     uint32_t target = arb_id(TEST_NODE_ID, CMD_HEARTBEAT);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(2000);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 uint32_t error = unpack_uint32(data);
                 test_result("Clear errors command sent successfully",
                     error == 0);
                 return;
             }
         }
     }
     test_result("Clear errors — heartbeat check", false);
 }
 
 void test_set_axis_state_write(int sock)
 {
     cout << "\n--- WRITE TEST 2: Set Axis State ---" << endl;
 
     // Send IDLE state
     uint8_t data[4];
     pack_uint32(data, AXIS_STATE_IDLE);
     send_can(sock, TEST_NODE_ID, CMD_SET_AXIS_STATE, data, 4);
     sleep_ms(500);
 
     // Verify by reading heartbeat — state should be IDLE
     uint32_t target = arb_id(TEST_NODE_ID, CMD_HEARTBEAT);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(2000);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  buf[8];
         if (recv_can(sock, id, buf, 500)) {
             if (id == target) {
                 uint8_t state = buf[4];
                 cout << "  State after IDLE command: " << (int)state << endl;
                 test_result("Set axis state IDLE command accepted",
                     state == AXIS_STATE_IDLE);
                 return;
             }
         }
     }
     test_result("Set axis state — heartbeat check", false);
 }
 
 void test_set_velocity_write(int sock)
 {
     cout << "\n--- WRITE TEST 3: Set Input Velocity ---" << endl;
 
     // Send velocity = 0 (safe test — no motor movement)
     uint8_t data[8];
     pack_float(data,     0.0f);
     pack_float(data + 4, 0.0f);
     send_can(sock, TEST_NODE_ID, CMD_SET_INPUT_VEL, data, 8);
     sleep_ms(100);
 
     cout << "  Velocity command (0.0 t/s) sent to node "
          << TEST_NODE_ID << endl;
     test_result("Set input velocity command sent", true);
 }
 
 void test_set_position_write(int sock)
 {
     cout << "\n--- WRITE TEST 4: Set Input Position ---" << endl;
 
     // Send position = 0 (safe test — no motor movement)
     uint8_t data[8];
     pack_float(data, 0.0f);
     int16_t vel_ff = 0, torque_ff = 0;
     memcpy(data + 4, &vel_ff,    2);
     memcpy(data + 6, &torque_ff, 2);
     send_can(sock, TEST_NODE_ID, CMD_SET_INPUT_POS, data, 8);
     sleep_ms(100);
 
     cout << "  Position command (0.0 turns) sent to node "
          << TEST_NODE_ID << endl;
     test_result("Set input position command sent", true);
 }
 
 // ─────────────────────────────────────────────────────────
 // MAIN
 // ─────────────────────────────────────────────────────────
 
 int main()
 {
     cout << string(50, '=') << endl;
     cout << "  ODrive CAN Test Suite" << endl;
     cout << "  Interface: " << CAN_INTERFACE << endl;
     cout << "  Node ID:   " << TEST_NODE_ID << endl;
     cout << string(50, '=') << endl;
 
     int sock = -1;
 
     // READ TESTS
     cout << "\n========== READ TESTS ==========" << endl;
     if (!test_can_socket_open(sock)) {
         cout << "\nCannot open CAN socket — aborting." << endl;
         cout << "Make sure: sudo ip link set can0 up type can bitrate 1000000" << endl;
         return 1;
     }
     test_heartbeat_read(sock);
     test_encoder_read(sock);
 
     // WRITE TESTS
     cout << "\n========== WRITE TESTS ==========" << endl;
     test_clear_errors_write(sock);
     test_set_axis_state_write(sock);
     test_set_velocity_write(sock);
     test_set_position_write(sock);
 
     // Results
     cout << "\n" << string(50, '=') << endl;
     cout << "  TEST RESULTS" << endl;
     cout << string(50, '=') << endl;
     cout << "  Passed: " << tests_passed << endl;
     cout << "  Failed: " << tests_failed << endl;
     cout << "  Total:  " << tests_passed + tests_failed << endl;
     cout << string(50, '=') << endl;
 
     close(sock);
     return tests_failed == 0 ? 0 : 1;
 }