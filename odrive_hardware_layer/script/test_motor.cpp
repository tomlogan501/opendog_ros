/**
 * ODrive Motor Test Script — C++
 * ================================
 * Test any combination of motors directly over SocketCAN.
 * No ROS2 required — pure CAN communication.
 *
 * Node ID mapping:
 *   ODrive 0 → axis0 = node 0,  axis1 = node 1
 *   ODrive 1 → axis0 = node 2,  axis1 = node 3
 *   ODrive 2 → axis0 = node 4,  axis1 = node 5
 *   ODrive 3 → axis0 = node 6,  axis1 = node 7
 *   ODrive 4 → axis0 = node 8,  axis1 = node 9
 *   ODrive 5 → axis0 = node 10, axis1 = node 11
 *
 * Usage:
 *   ./test_motor 0          # test one motor
 *   ./test_motor 0 1        # test one full ODrive
 *   ./test_motor 0 2 4      # test one leg
 *   ./test_motor 0 1 2 3 4 5 6 7 8 9 10 11  # test all
 *
 * Compile:
 *   g++ test_motor.cpp -o test_motor
 *
 * Author: Ahsan Ali
 * Date:   May 2026
 */

 #include <iostream>
 #include <vector>
 #include <string>
 #include <cstring>
 #include <cstdint>
 #include <cmath>
 #include <unistd.h>
 #include <net/if.h>
 #include <sys/socket.h>
 #include <sys/ioctl.h>
 #include <sys/select.h>
 #include <linux/can.h>
 #include <linux/can/raw.h>
 #include <chrono>
 #include <thread>
 #include <map>
 
 using namespace std;
 
 // ─────────────────────────────────────────────────────────
 // NODE ID MAPPING
 // ─────────────────────────────────────────────────────────
 
 map<int, string> NODE_MAP = {
     {0,  "ODrive0 axis0"},
     {1,  "ODrive0 axis1"},
     {2,  "ODrive1 axis0"},
     {3,  "ODrive1 axis1"},
     {4,  "ODrive2 axis0"},
     {5,  "ODrive2 axis1"},
     {6,  "ODrive3 axis0"},
     {7,  "ODrive3 axis1"},
     {8,  "ODrive4 axis0"},
     {9,  "ODrive4 axis1"},
     {10, "ODrive5 axis0"},
     {11, "ODrive5 axis1"},
 };
 
 // ─────────────────────────────────────────────────────────
 // ODrive CANSimple COMMAND IDs
 // ─────────────────────────────────────────────────────────
 
 constexpr uint8_t CMD_HEARTBEAT          = 0x01;
 constexpr uint8_t CMD_ESTOP              = 0x02;
 constexpr uint8_t CMD_SET_AXIS_STATE     = 0x07;
 constexpr uint8_t CMD_GET_ENCODER_EST    = 0x09;
 constexpr uint8_t CMD_SET_CTRL_MODE      = 0x0B;
 constexpr uint8_t CMD_SET_INPUT_POS      = 0x0C;
 constexpr uint8_t CMD_SET_INPUT_VEL      = 0x0D;
 constexpr uint8_t CMD_GET_SENSORLESS_EST = 0x15;
 constexpr uint8_t CMD_CLEAR_ERRORS       = 0x18;
 
 // ─────────────────────────────────────────────────────────
 // ODrive AXIS STATES
 // ─────────────────────────────────────────────────────────
 
 constexpr uint32_t AXIS_STATE_IDLE                = 1;
 constexpr uint32_t AXIS_STATE_MOTOR_CALIBRATION   = 4;
 constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
 
 // ─────────────────────────────────────────────────────────
 // ODrive CONTROL MODES
 // ─────────────────────────────────────────────────────────
 
 constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
 constexpr uint32_t CONTROL_MODE_POSITION_CONTROL = 3;
 constexpr uint32_t INPUT_MODE_PASSTHROUGH         = 1;
 constexpr uint32_t INPUT_MODE_TRAP_TRAJ           = 5;
 
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
 
 void pack_float(uint8_t * buf, float val)
 {
     memcpy(buf, &val, 4);
 }
 
 void pack_uint32(uint8_t * buf, uint32_t val)
 {
     memcpy(buf, &val, 4);
 }
 
 float unpack_float(uint8_t * buf)
 {
     float val;
     memcpy(&val, buf, 4);
     return val;
 }
 
 uint32_t unpack_uint32(uint8_t * buf)
 {
     uint32_t val;
     memcpy(&val, buf, 4);
     return val;
 }
 
 // ─────────────────────────────────────────────────────────
 // CAN SOCKET
 // ─────────────────────────────────────────────────────────
 
 int can_open(const string & interface)
 {
     int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
     if (sock < 0) { perror("socket"); return -1; }
 
     struct ifreq ifr;
     strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
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
 
 // ─────────────────────────────────────────────────────────
 // SEND / RECEIVE
 // ─────────────────────────────────────────────────────────
 
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
 // SEND COMMANDS
 // ─────────────────────────────────────────────────────────
 
 void set_axis_state(int sock, int node_id, uint32_t state)
 {
     uint8_t data[4];
     pack_uint32(data, state);
     send_can(sock, node_id, CMD_SET_AXIS_STATE, data, 4);
 
     map<uint32_t,string> names = {
         {1,"IDLE"},{4,"MOTOR_CALIBRATION"},{8,"CLOSED_LOOP"}};
     cout << "  [node " << node_id << "] → "
          << names[state] << endl;
 }
 
 void set_control_mode(int sock, int node_id,
                       uint32_t ctrl_mode, uint32_t input_mode)
 {
     uint8_t data[8];
     pack_uint32(data,     ctrl_mode);
     pack_uint32(data + 4, input_mode);
     send_can(sock, node_id, CMD_SET_CTRL_MODE, data, 8);
 }
 
 void set_input_velocity(int sock, int node_id,
                         float velocity, float torque_ff = 0.0f)
 {
     uint8_t data[8];
     pack_float(data,     velocity);
     pack_float(data + 4, torque_ff);
     send_can(sock, node_id, CMD_SET_INPUT_VEL, data, 8);
     cout << "  [node " << node_id << "] → velocity: "
          << velocity << " turns/sec" << endl;
 }
 
 void set_input_position(int sock, int node_id, float position,
                         int16_t vel_ff = 0, int16_t torque_ff = 0)
 {
     uint8_t data[8];
     pack_float(data, position);
     memcpy(data + 4, &vel_ff,    2);
     memcpy(data + 6, &torque_ff, 2);
     send_can(sock, node_id, CMD_SET_INPUT_POS, data, 8);
     cout << "  [node " << node_id << "] → position: "
          << position << " turns" << endl;
 }
 
 void clear_errors(int sock, int node_id)
 {
     send_can(sock, node_id, CMD_CLEAR_ERRORS, nullptr, 0);
 }
 
 void estop_all(int sock, const vector<int> & node_ids)
 {
     for (int id : node_ids)
         send_can(sock, id, CMD_ESTOP, nullptr, 0);
     cout << "  ESTOP sent to all nodes" << endl;
 }
 
 // ─────────────────────────────────────────────────────────
 // READ DATA
 // ─────────────────────────────────────────────────────────
 
 bool get_heartbeat(int sock, int node_id,
                    uint32_t & axis_error, uint8_t & axis_state,
                    int timeout_ms = 3000)
 {
     uint32_t target = arb_id(node_id, CMD_HEARTBEAT);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(timeout_ms);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 axis_error = unpack_uint32(data);
                 axis_state = data[4];
                 return true;
             }
         }
     }
     return false;
 }
 
 bool get_encoder_estimates(int sock, int node_id,
                            float & pos, float & vel,
                            int timeout_ms = 2000)
 {
     uint32_t target = arb_id(node_id, CMD_GET_ENCODER_EST);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(timeout_ms);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 pos = unpack_float(data);
                 vel = unpack_float(data + 4);
                 return true;
             }
         }
     }
     return false;
 }
 
 bool get_sensorless_estimates(int sock, int node_id,
                               float & pos, float & vel,
                               int timeout_ms = 2000)
 {
     uint32_t target = arb_id(node_id, CMD_GET_SENSORLESS_EST);
     auto deadline = chrono::steady_clock::now() +
                     chrono::milliseconds(timeout_ms);
 
     while (chrono::steady_clock::now() < deadline) {
         uint32_t id;
         uint8_t  data[8];
         if (recv_can(sock, id, data, 500)) {
             if (id == target) {
                 pos = unpack_float(data);
                 vel = unpack_float(data + 4);
                 return true;
             }
         }
     }
     return false;
 }
 
 // ─────────────────────────────────────────────────────────
 // TEST SEQUENCE FOR ONE NODE
 // ─────────────────────────────────────────────────────────
 
 bool test_node(int sock, int node_id)
 {
     string name = NODE_MAP.count(node_id)
         ? NODE_MAP[node_id] : "node " + to_string(node_id);
 
     cout << "\n" << string(50, '=') << endl;
     cout << "  Testing " << name << " (node " << node_id << ")" << endl;
     cout << string(50, '=') << endl;
 
     // Step 1 — Heartbeat
     cout << "\n[1] Checking heartbeat..." << endl;
     uint32_t axis_error;
     uint8_t  axis_state;
     if (!get_heartbeat(sock, node_id, axis_error, axis_state, 3000)) {
         cout << "  [node " << node_id << "] ERROR: No heartbeat" << endl;
         return false;
     }
     cout << "  [node " << node_id << "] Alive — state="
          << (int)axis_state << " error=0x" << hex << axis_error
          << dec << endl;
 
     // Step 2 — Clear errors
     cout << "\n[2] Clearing errors..." << endl;
     clear_errors(sock, node_id);
     sleep_ms(300);
 
     // Step 3 — Calibrate
     cout << "\n[3] Calibrating motor (listen for beep)..." << endl;
     set_axis_state(sock, node_id, AXIS_STATE_MOTOR_CALIBRATION);
     sleep_ms(8000);
 
     if (!get_heartbeat(sock, node_id, axis_error, axis_state, 3000)) {
         cout << "  [node " << node_id << "] No heartbeat after calibration" << endl;
         return false;
     }
     if (axis_error != 0) {
         cout << "  [node " << node_id << "] Calibration ERROR: 0x"
              << hex << axis_error << dec << endl;
         return false;
     }
     cout << "  [node " << node_id << "] Calibration done" << endl;
     clear_errors(sock, node_id);
     sleep_ms(300);
 
     // ── VELOCITY TEST ──────────────────────────────────
     cout << "\n[4] VELOCITY CONTROL TEST" << endl;
     set_control_mode(sock, node_id,
                      CONTROL_MODE_VELOCITY_CONTROL,
                      INPUT_MODE_PASSTHROUGH);
     sleep_ms(200);
 
     set_axis_state(sock, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
     sleep_ms(2000);
 
     vector<float> velocities = {2.0f, 4.0f, -2.0f, 0.0f};
     for (float vel : velocities) {
         set_input_velocity(sock, node_id, vel);
         sleep_ms(2000);
         float pos, v;
         if (get_sensorless_estimates(sock, node_id, pos, v, 1000)) {
             cout << "    feedback → pos=" << pos
                  << " turns  vel=" << v << " t/s" << endl;
         }
     }
 
     set_input_velocity(sock, node_id, 0.0f);
     sleep_ms(1000);
     set_axis_state(sock, node_id, AXIS_STATE_IDLE);
     sleep_ms(1000);
 
     // ── POSITION TEST ──────────────────────────────────
     cout << "\n[5] POSITION CONTROL TEST" << endl;
 
     clear_errors(sock, node_id);
     set_axis_state(sock, node_id, AXIS_STATE_MOTOR_CALIBRATION);
     sleep_ms(8000);
     clear_errors(sock, node_id);
     sleep_ms(300);
 
     set_control_mode(sock, node_id,
                      CONTROL_MODE_POSITION_CONTROL,
                      INPUT_MODE_TRAP_TRAJ);
     sleep_ms(200);
 
     set_axis_state(sock, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
     sleep_ms(2000);
 
     vector<float> positions = {1.0f, 2.0f, -1.0f, 0.0f};
     for (float pos_target : positions) {
         set_input_position(sock, node_id, pos_target);
         sleep_ms(3000);
         float pos, vel;
         if (get_encoder_estimates(sock, node_id, pos, vel, 1000)) {
             cout << "    feedback → pos=" << pos
                  << " turns  vel=" << vel << " t/s" << endl;
         } else {
             cout << "    feedback → no encoder data (sensorless mode)" << endl;
         }
     }
 
     set_input_position(sock, node_id, 0.0f);
     sleep_ms(1000);
     set_axis_state(sock, node_id, AXIS_STATE_IDLE);
 
     cout << "\n  [node " << node_id << "] ✓ Test complete" << endl;
     return true;
 }
 
 // ─────────────────────────────────────────────────────────
 // MAIN
 // ─────────────────────────────────────────────────────────
 
 int main(int argc, char * argv[])
 {
     if (argc < 2) {
         cout << "Usage: ./test_motor <node_id> [node_id] ..." << endl;
         cout << "Example: ./test_motor 0 1 2" << endl;
         return 1;
     }
 
     vector<int> node_ids;
     for (int i = 1; i < argc; i++)
         node_ids.push_back(stoi(argv[i]));
 
     cout << string(50, '=') << endl;
     cout << "  ODrive Motor Test — SocketCAN C++" << endl;
     cout << "  Testing nodes: ";
     for (int id : node_ids) cout << id << " ";
     cout << endl;
     cout << string(50, '=') << endl;
 
     // Open CAN socket
     int sock = can_open("can0");
     if (sock < 0) {
         cerr << "Failed to open CAN socket" << endl;
         return 1;
     }
     cout << "CAN socket opened." << endl;
 
     map<int, bool> results;
 
     try {
         for (int node_id : node_ids) {
             results[node_id] = test_node(sock, node_id);
         }
     } catch (...) {
         cout << "\nInterrupted — stopping all motors..." << endl;
         estop_all(sock, node_ids);
     }
 
     close(sock);
 
     // Summary
     cout << "\n" << string(50, '=') << endl;
     cout << "  TEST SUMMARY" << endl;
     cout << string(50, '=') << endl;
     for (auto & [node_id, success] : results) {
         string name = NODE_MAP.count(node_id)
             ? NODE_MAP[node_id] : to_string(node_id);
         cout << "  Node " << node_id << " ("
              << name << ") → "
              << (success ? "PASS" : "FAIL") << endl;
     }
     cout << string(50, '=') << endl;
 
     return 0;
 }