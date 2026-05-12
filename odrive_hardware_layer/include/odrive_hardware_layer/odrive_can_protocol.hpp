#pragma once

#include <cstdint>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace odrive_hardware_layer
{

// ─────────────────────────────────────────────────────────
// ODrive CANSimple COMMAND IDs
// Reference: docs.odriverobotics.com/v/0.5.6/can-protocol.html
// Arbitration ID = (node_id << 5) | cmd_id
// ─────────────────────────────────────────────────────────

constexpr uint8_t CMD_HEARTBEAT          = 0x01;
constexpr uint8_t CMD_ESTOP              = 0x02;
constexpr uint8_t CMD_GET_MOTOR_ERROR    = 0x03;
constexpr uint8_t CMD_GET_ENCODER_ERROR  = 0x04;
constexpr uint8_t CMD_SET_AXIS_STATE     = 0x07;
constexpr uint8_t CMD_GET_ENCODER_EST    = 0x09;
constexpr uint8_t CMD_SET_CTRL_MODE      = 0x0B;
constexpr uint8_t CMD_SET_INPUT_POS      = 0x0C;
constexpr uint8_t CMD_SET_INPUT_VEL      = 0x0D;
constexpr uint8_t CMD_SET_INPUT_TORQUE   = 0x0E;
constexpr uint8_t CMD_SET_LIMITS         = 0x0F;
constexpr uint8_t CMD_GET_IQ             = 0x14;
constexpr uint8_t CMD_GET_SENSORLESS_EST = 0x15;
constexpr uint8_t CMD_REBOOT             = 0x16;
constexpr uint8_t CMD_GET_BUS_VOLTAGE    = 0x17;
constexpr uint8_t CMD_CLEAR_ERRORS       = 0x18;
constexpr uint8_t CMD_SET_POS_GAIN       = 0x1A;
constexpr uint8_t CMD_SET_VEL_GAINS      = 0x1B;
constexpr uint8_t CMD_GET_CTRL_ERROR     = 0x1D;

// ─────────────────────────────────────────────────────────
// ODrive AXIS STATES
// ─────────────────────────────────────────────────────────

constexpr uint32_t AXIS_STATE_IDLE                      = 1;
constexpr uint32_t AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3;
constexpr uint32_t AXIS_STATE_MOTOR_CALIBRATION         = 4;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL       = 8;

// ─────────────────────────────────────────────────────────
// ODrive CONTROL MODES
// ─────────────────────────────────────────────────────────

constexpr uint32_t CONTROL_MODE_VOLTAGE_CONTROL  = 0;
constexpr uint32_t CONTROL_MODE_TORQUE_CONTROL   = 1;
constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
constexpr uint32_t CONTROL_MODE_POSITION_CONTROL = 3;

// ─────────────────────────────────────────────────────────
// ODrive INPUT MODES
// ─────────────────────────────────────────────────────────

constexpr uint32_t INPUT_MODE_INACTIVE    = 0;
constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;
constexpr uint32_t INPUT_MODE_VEL_RAMP    = 2;
constexpr uint32_t INPUT_MODE_POS_FILTER  = 3;
constexpr uint32_t INPUT_MODE_TRAP_TRAJ   = 5;
constexpr uint32_t INPUT_MODE_TORQUE_RAMP = 6;

// ─────────────────────────────────────────────────────────
// HELPER
// ─────────────────────────────────────────────────────────

inline uint32_t arb_id(int node_id, uint8_t cmd_id)
{
  return (node_id << 5) | cmd_id;
}

// ─────────────────────────────────────────────────────────
// FUNCTION DECLARATIONS
// ─────────────────────────────────────────────────────────

int  can_open(const std::string & interface);
void can_close(int sock);

void send_can(int sock, int node_id, uint8_t cmd_id, uint8_t * data, uint8_t len);
bool recv_can(int sock, uint32_t & arb_id_out, uint8_t * data_out, int timeout_ms = 1000);

// Send commands
void set_axis_state   (int sock, int node_id, uint32_t state);
void set_control_mode (int sock, int node_id, uint32_t ctrl_mode, uint32_t input_mode);
void set_input_velocity(int sock, int node_id, float velocity, float torque_ff = 0.0f);
void set_input_position(int sock, int node_id, float position, int16_t vel_ff = 0, int16_t torque_ff = 0);
void set_input_torque  (int sock, int node_id, float torque);
void set_limits        (int sock, int node_id, float vel_limit, float current_limit);
void clear_errors      (int sock, int node_id);
void estop             (int sock, int node_id);

// Read data
bool get_heartbeat        (int sock, int node_id, uint32_t & error, uint8_t & state, int timeout_ms = 3000);
bool get_encoder_estimates(int sock, int node_id, float & pos, float & vel, int timeout_ms = 2000);
bool get_iq               (int sock, int node_id, float & iq_set, float & iq_meas, int timeout_ms = 2000);

}  // namespace odrive_hardware_layer