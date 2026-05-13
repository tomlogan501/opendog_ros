#include <iostream>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std;


// CONFIGURATION
const char* CAN_INTERFACE = "can0";
const int   NODE_ID       = 0;


// ODrive CANSimple COMMAND IDs
const int CMD_HEARTBEAT        = 0x001;
const int CMD_SET_AXIS_STATE   = 0x007;
const int CMD_GET_ENCODER_EST  = 0x009;
const int CMD_SET_CTRL_MODE    = 0x00B;
const int CMD_SET_INPUT_POS    = 0x00C;
const int CMD_SET_INPUT_VEL    = 0x00D;
const int CMD_SET_INPUT_TORQUE = 0x00E;
const int CMD_CLEAR_ERRORS     = 0x018;


// ODrive AXIS STATES
const int AXIS_STATE_IDLE              = 1;
const int AXIS_STATE_MOTOR_CALIBRATION = 4;
const int AXIS_STATE_CLOSED_LOOP       = 8;


// ODrive CONTROL MODES
const int CONTROL_MODE_TORQUE    = 1;
const int CONTROL_MODE_VELOCITY  = 2;
const int CONTROL_MODE_POSITION  = 3;
const int INPUT_MODE_PASSTHROUGH = 1;

// Global socket
int sock;

// HELPER

int arb_id(int node_id, int cmd_id) {
    return (node_id << 5) | cmd_id;
}

void sleep_ms(int ms) {
    this_thread::sleep_for(chrono::milliseconds(ms));
}

// SEND COMMANDS  (Host → ODrive)

void send_can(int cmd_id, uint8_t* data, int len) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = arb_id(NODE_ID, cmd_id);
    frame.can_dlc = len;
    if (data && len > 0)
        memcpy(frame.data, data, len);
    write(sock, &frame, sizeof(frame));
}

void set_axis_state(int state) {
    uint32_t s = state;
    send_can(CMD_SET_AXIS_STATE, (uint8_t*)&s, 4);
    string name = "UNKNOWN";
    if (state == 1) name = "IDLE";
    if (state == 4) name = "MOTOR_CALIBRATION";
    if (state == 8) name = "CLOSED_LOOP";
    cout << "  [SEND] Axis state → " << name << endl;
}

void set_control_mode(int ctrl_mode, int input_mode) {
    uint32_t data[2] = {(uint32_t)ctrl_mode, (uint32_t)input_mode};
    send_can(CMD_SET_CTRL_MODE, (uint8_t*)data, 8);
    cout << "  [SEND] Control mode → " << ctrl_mode
         << ", Input mode → " << input_mode << endl;
}

void set_input_velocity(float vel, float torque_ff = 0.0f) {
    float data[2] = {vel, torque_ff};
    send_can(CMD_SET_INPUT_VEL, (uint8_t*)data, 8);
    cout << "  [SEND] Velocity → " << vel << " turns/sec" << endl;
}

void set_input_position(float pos, int16_t vel_ff = 0,
                        int16_t torque_ff = 0) {
    uint8_t data[8];
    memcpy(data,     &pos,      4);
    memcpy(data + 4, &vel_ff,   2);
    memcpy(data + 6, &torque_ff,2);
    send_can(CMD_SET_INPUT_POS, data, 8);
    cout << "  [SEND] Position → " << pos << " turns" << endl;
}

void set_input_torque(float torque) {
    send_can(CMD_SET_INPUT_TORQUE, (uint8_t*)&torque, 4);
    cout << "  [SEND] Torque → " << torque << " Nm" << endl;
}

void clear_errors() {
    send_can(CMD_CLEAR_ERRORS, nullptr, 0);
    cout << "  [SEND] Clear errors" << endl;
}

// RETRIEVE DATA  (ODrive → Host)

bool get_heartbeat(uint32_t& error, uint8_t& state,
                   int timeout_ms = 3000) {
    struct can_frame frame;
    int target = arb_id(NODE_ID, CMD_HEARTBEAT);
    auto deadline = chrono::steady_clock::now() +
                    chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = read(sock, &frame, sizeof(frame));
        if (n > 0 && (int)frame.can_id == target) {
            memcpy(&error, frame.data, 4);
            state = frame.data[4];
            return true;
        }
    }
    return false;
}

bool get_encoder_estimates(float& pos, float& vel,
                           int timeout_ms = 2000) {
    struct can_frame frame;
    int target = arb_id(NODE_ID, CMD_GET_ENCODER_EST);
    auto deadline = chrono::steady_clock::now() +
                    chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = read(sock, &frame, sizeof(frame));
        if (n > 0 && (int)frame.can_id == target) {
            memcpy(&pos, frame.data,     4);
            memcpy(&vel, frame.data + 4, 4);
            return true;
        }
    }
    return false;
}

void print_status() {
    uint32_t error;
    uint8_t  state;
    if (get_heartbeat(error, state, 2000)) {
        string name = "UNKNOWN";
        if (state == 1) name = "IDLE";
        if (state == 4) name = "MOTOR_CALIBRATION";
        if (state == 8) name = "CLOSED_LOOP";
        cout << "  State : " << name << " (" << (int)state << ")" << endl;
        cout << "  Error : 0x" << hex << error
             << (error == 0 ? " OK" : " ERROR!") << dec << endl;
    } else {
        cout << "  Heartbeat: no response" << endl;
    }
    float pos, vel;
    if (get_encoder_estimates(pos, vel, 1000)) {
        cout << "  Pos   : " << pos << " turns" << endl;
        cout << "  Vel   : " << vel << " turns/sec" << endl;
    }
  
}

// MAIN


int main() {
    cout << "  ODrive CAN Motor Test — C++" << endl;
    cout << "  SocketCAN | DSD Tech SH-C30G" << endl;

    // Open SocketCAN socket
    cout << "\nConnecting to " << CAN_INTERFACE << "..." << endl;
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        cerr << "Failed to open socket." << endl;
        return 1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, CAN_INTERFACE);
    ioctl(sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    cout << "Connected to " << CAN_INTERFACE << endl;

    // Wait for heartbeat
    cout << "\nWaiting for ODrive heartbeat..." << endl;
    uint32_t axis_error;
    uint8_t  axis_state;
    if (!get_heartbeat(axis_error, axis_state, 5000)) {
        cerr << "No heartbeat — check wiring and node_id." << endl;
        close(sock);
        return 1;
    }
    cout << "ODrive alive! State=" << (int)axis_state
         << ", Error=0x" << hex << axis_error << dec << endl;

    // Clear errors
    clear_errors();
    sleep_ms(500);

    // Calibrate
    cout << "\nCalibrating motor (listen for beep)..." << endl;
    set_axis_state(AXIS_STATE_MOTOR_CALIBRATION);
    sleep_ms(8000);

    // Check calibration
    if (!get_heartbeat(axis_error, axis_state, 3000)) {
        cerr << "No heartbeat after calibration." << endl;
        close(sock);
        return 1;
    }
    if (axis_error != 0) {
        cerr << "Calibration failed: 0x" << hex << axis_error << endl;
        close(sock);
        return 1;
    }
    cout << "Calibration done!" << endl;

    // Clear post-calibration errors
    clear_errors();
    sleep_ms(500);

    // Set velocity control mode
    set_control_mode(CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH);
    sleep_ms(300);

    // Enter closed loop
    cout << "\nEntering closed loop..." << endl;
    set_axis_state(AXIS_STATE_CLOSED_LOOP);
    sleep_ms(2000);

    // Status before spinning
    print_status();

    // Spin
    cout << "Spinning at 2.0 turns/sec for 5 seconds..." << endl;
    set_input_velocity(2.0f);

    // Read feedback while spinning
    cout << "\nReading feedback:" << endl;
    for (int i = 0; i < 5; i++) {
        set_input_velocity(2.0f);
        uint32_t err;
        uint8_t  st;
        if (get_heartbeat(err, st, 1000)) {
            string name = st == 8 ? "CLOSED_LOOP" : "IDLE";
            cout << "  [" << (i+1) << "] State=" << name
                 << " Error=0x" << hex << err << dec << endl;
        }
        sleep_ms(800);
    }

    // Stop
    cout << "\nStopping motor..." << endl;
    set_input_velocity(0.0f);
    sleep_ms(1000);
    set_axis_state(AXIS_STATE_IDLE);

    // Final status
    print_status();

    close(sock);
    
    cout << "  Test complete." << endl;
  

    return 0;
}
