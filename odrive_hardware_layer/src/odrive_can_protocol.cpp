#include "odrive_hardware_layer/odrive_can_protocol.hpp"

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>

namespace odrive_hardware_layer
{

// ─────────────────────────────────────────────────────────
// SOCKET OPEN / CLOSE
// ─────────────────────────────────────────────────────────

int can_open(const std::string & interface)
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    perror("socket");
    return -1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl");
    close(sock);
    return -1;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(sock);
    return -1;
  }

  return sock;
}

void can_close(int sock)
{
  close(sock);
}

// ─────────────────────────────────────────────────────────
// SEND / RECEIVE
// ─────────────────────────────────────────────────────────

void send_can(int sock, int node_id, uint8_t cmd_id, uint8_t * data, uint8_t len)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id  = arb_id(node_id, cmd_id);
  frame.can_dlc = len;
  if (data && len > 0) {
    std::memcpy(frame.data, data, len);
  }

  write(sock, &frame, sizeof(frame));
}

bool recv_can(int sock, uint32_t & arb_id_out, uint8_t * data_out, int timeout_ms)
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(sock, &read_fds);

  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(sock + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret <= 0) return false;

  struct can_frame frame;
  if (read(sock, &frame, sizeof(frame)) < 0) return false;

  arb_id_out = frame.can_id;
  std::memcpy(data_out, frame.data, 8);
  return true;
}

// ─────────────────────────────────────────────────────────
// FLOAT / UINT32 CONVERSION HELPERS
// ─────────────────────────────────────────────────────────

static void pack_float(uint8_t * buf, float val)
{
  std::memcpy(buf, &val, 4);
}

static void pack_uint32(uint8_t * buf, uint32_t val)
{
  std::memcpy(buf, &val, 4);
}

static float unpack_float(uint8_t * buf)
{
  float val;
  std::memcpy(&val, buf, 4);
  return val;
}

static uint32_t unpack_uint32(uint8_t * buf)
{
  uint32_t val;
  std::memcpy(&val, buf, 4);
  return val;
}

// ─────────────────────────────────────────────────────────
// SEND COMMANDS  (Host → ODrive)
// ─────────────────────────────────────────────────────────

void set_axis_state(int sock, int node_id, uint32_t state)
{
  uint8_t data[4];
  pack_uint32(data, state);
  send_can(sock, node_id, CMD_SET_AXIS_STATE, data, 4);
}

void set_control_mode(int sock, int node_id, uint32_t ctrl_mode, uint32_t input_mode)
{
  uint8_t data[8];
  pack_uint32(data,     ctrl_mode);
  pack_uint32(data + 4, input_mode);
  send_can(sock, node_id, CMD_SET_CTRL_MODE, data, 8);
}

void set_input_velocity(int sock, int node_id, float velocity, float torque_ff)
{
  uint8_t data[8];
  pack_float(data,     velocity);
  pack_float(data + 4, torque_ff);
  send_can(sock, node_id, CMD_SET_INPUT_VEL, data, 8);
}

void set_input_position(int sock, int node_id, float position, int16_t vel_ff, int16_t torque_ff)
{
  uint8_t data[8];
  pack_float(data, position);
  std::memcpy(data + 4, &vel_ff,    2);
  std::memcpy(data + 6, &torque_ff, 2);
  send_can(sock, node_id, CMD_SET_INPUT_POS, data, 8);
}

void set_input_torque(int sock, int node_id, float torque)
{
  uint8_t data[4];
  pack_float(data, torque);
  send_can(sock, node_id, CMD_SET_INPUT_TORQUE, data, 4);
}

void set_limits(int sock, int node_id, float vel_limit, float current_limit)
{
  uint8_t data[8];
  pack_float(data,     vel_limit);
  pack_float(data + 4, current_limit);
  send_can(sock, node_id, CMD_SET_LIMITS, data, 8);
}

void clear_errors(int sock, int node_id)
{
  send_can(sock, node_id, CMD_CLEAR_ERRORS, nullptr, 0);
}

void estop(int sock, int node_id)
{
  send_can(sock, node_id, CMD_ESTOP, nullptr, 0);
}

// ─────────────────────────────────────────────────────────
// RETRIEVE DATA  (ODrive → Host)
// ─────────────────────────────────────────────────────────

bool get_heartbeat(int sock, int node_id, uint32_t & axis_error,
                   uint8_t & axis_state, int timeout_ms)
{
  uint32_t target = arb_id(node_id, CMD_HEARTBEAT);
  uint32_t id;
  uint8_t  data[8];

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
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

bool get_encoder_estimates(int sock, int node_id, float & pos,
                           float & vel, int timeout_ms)
{
  uint32_t target = arb_id(node_id, CMD_GET_ENCODER_EST);
  uint32_t id;
  uint8_t  data[8];

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
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

bool get_iq(int sock, int node_id, float & iq_set,
            float & iq_meas, int timeout_ms)
{
  uint32_t target = arb_id(node_id, CMD_GET_IQ);
  uint32_t id;