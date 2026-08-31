// Copyright 2026 Reebot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odrive_can.hpp"

#include <chrono>
#include <thread>
#include <unistd.h>        // ::write, ::read, close
#include <sys/ioctl.h>    // ioctl
#include <net/if.h>       // struct ifreq, IFNAMSIZ
#include <sys/socket.h>   // socket, bind, send/recv
#include <sys/select.h>   // select
#include <linux/can.h>    // struct can_frame
#include <errno.h>
#include <cstring>        // std::memcpy, std::strerror, strncpy, memset
#include <cstdint>        // INT16_MIN, INT16_MAX
#include <cmath>          // std::lround
#include <algorithm>      // std::clamp
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include "odrive_endpoints.hpp"

namespace odrive
{

ODriveCAN::ODriveCAN() : can_socket_(-1), running_(false) {}

ODriveCAN::~ODriveCAN()
{
  // Stop the receive thread
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

int ODriveCAN::init(const std::vector<std::vector<int64_t>>& node_ids, const std::string& can_interface)
{
  can_interface_ = can_interface;

  // Create socket
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error while opening socket: %s", std::strerror(errno));
    return -1;
  }

  // Get interface index
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  // use strncpy to avoid overflow
  strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error getting interface index for %s: %s",
                 can_interface_.c_str(), std::strerror(errno));
    ::close(can_socket_);
    can_socket_ = -1;
    return -1;
  }

  // Bind socket to interface
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error binding socket to interface %s: %s",
                 can_interface_.c_str(), std::strerror(errno));
    ::close(can_socket_);
    can_socket_ = -1;
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveCAN"), "CAN interface initialized on %s", can_interface_.c_str());

  // Store node IDs
  for (const auto& group : node_ids) {
    for (auto node_id : group) {
      node_id_map_[node_id] = node_id;
      RCLCPP_INFO(rclcpp::get_logger("ODriveCAN"), "Node ID %ld added", node_id);
    }
  }

  // Start the CAN receive thread
  running_ = true;
  receive_thread_ = std::thread(&ODriveCAN::receive_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("ODriveCAN"), "CAN receive thread started");

  return 0;
}

// Read/write template implementation for compatibility with the existing interface
template <typename T>
int ODriveCAN::read(int64_t node_id, short endpoint_id, T& value)
{
  // For compatibility with existing code, we implement a generic read
  // This method is less efficient than the specific methods
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  // Envoi de la commande de lecture
  uint32_t can_id = endpoint_to_can_id(node_id, endpoint_id, true);
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id | CAN_RTR_FLAG;
  frame.can_dlc = 0; // Read with no data
  
  if (canSend(frame) != 0) {
    return -1;
  }
  
  // Wait for the response (simplified)
  struct can_frame response;
  std::memset(&response, 0, sizeof(response));
  if (canReceive(response, 100) == 0) {
    // Basic response handling
    if (sizeof(T) <= static_cast<size_t>(response.can_dlc)) {
      std::memcpy(&value, response.data, sizeof(T));
      return 0;
    } else {
      // response too short
      return -1;
    }
  }
  
  return -1;
}

template <typename T>
int ODriveCAN::write(int64_t node_id, short endpoint_id, const T& value)
{
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  uint32_t can_id = endpoint_to_can_id(node_id, endpoint_id, false);
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;

  // Protect against writing more than 8 bytes (classic CAN frame).
  // Use a unique name to avoid macro collision with kernel headers.
  constexpr size_t ODRIVE_LOCAL_CAN_MAX_DLEN = 8;
  if (sizeof(T) > ODRIVE_LOCAL_CAN_MAX_DLEN) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"),
                 "Attempt to write %zu bytes in CAN frame (max %zu). Aborting.",
                 static_cast<size_t>(sizeof(T)),
                 static_cast<size_t>(ODRIVE_LOCAL_CAN_MAX_DLEN));
    return -1;
  }

  frame.can_dlc = static_cast<uint8_t>(sizeof(T));
  std::memcpy(frame.data, &value, sizeof(T));
  
  return canSend(frame);
}

int ODriveCAN::call(int64_t node_id, short endpoint_id)
{
  // For a call, we just send the command with no data
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  uint32_t can_id = endpoint_to_can_id(node_id, endpoint_id, false);
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id | CAN_RTR_FLAG;
  frame.can_dlc = 0;
  
  return canSend(frame);
}

// CAN-specific function implementations
bool ODriveCAN::send_set_axis_state(int64_t node_id, int32_t requested_state)
{
  invalidate_heartbeat(node_id);
  uint8_t data[8] = {0};
  int32_to_bytes(requested_state, data);
  return send_can_message(node_id, 0x007, data, 8);
}

bool ODriveCAN::send_clear_errors(int64_t node_id)
{
  invalidate_heartbeat(node_id);
  return send_can_message(node_id, 0x018, nullptr, 0);
}

bool ODriveCAN::send_set_controller_mode(int64_t node_id, int32_t control_mode, int32_t input_mode)
{
  uint8_t data[8];
  int32_to_bytes(control_mode, data);
  int32_to_bytes(input_mode, data + 4);

  return send_can_message(node_id, 0x00B, data, 8);
}

bool ODriveCAN::send_set_input_pos(int64_t node_id, float position, float velocity_feedforward, float torque_feedforward)
{
  // Set_Input_Pos wire layout (ODrive CANSimple, matches odrive-cansimple.dbc):
  //   bytes 0-3: Input_Pos    (float32, turns)
  //   bytes 4-5: Vel_FF       (int16, scale 0.001 -> turns/s)
  //   bytes 6-7: Torque_FF    (int16, scale 0.001 -> Nm)
  // Both feedforward fields fit in the 8-byte classic CAN frame; no data is dropped.
  uint8_t data[8];
  float_to_bytes(position, data);

  int32_t vel_ff_scaled = static_cast<int32_t>(std::lround(velocity_feedforward / 0.001f));
  int32_t torque_ff_scaled = static_cast<int32_t>(std::lround(torque_feedforward / 0.001f));
  vel_ff_scaled = std::clamp(vel_ff_scaled, static_cast<int32_t>(INT16_MIN), static_cast<int32_t>(INT16_MAX));
  torque_ff_scaled = std::clamp(torque_ff_scaled, static_cast<int32_t>(INT16_MIN), static_cast<int32_t>(INT16_MAX));

  int16_t vel_ff_i16 = static_cast<int16_t>(vel_ff_scaled);
  int16_t torque_ff_i16 = static_cast<int16_t>(torque_ff_scaled);
  std::memcpy(data + 4, &vel_ff_i16, sizeof(int16_t));
  std::memcpy(data + 6, &torque_ff_i16, sizeof(int16_t));

  return send_can_message(node_id, 0x00C, data, 8);
}

bool ODriveCAN::send_set_input_vel(int64_t node_id, float velocity, float torque_feedforward)
{
  uint8_t data[8];
  float_to_bytes(velocity, data);
  float_to_bytes(torque_feedforward, data + 4);
  
  return send_can_message(node_id, 0x00D, data, 8);
}

bool ODriveCAN::send_set_input_torque(int64_t node_id, float torque)
{
  uint8_t data[4];
  float_to_bytes(torque, data);
  
  return send_can_message(node_id, 0x00E, data, 4);
}

bool ODriveCAN::get_encoder_estimates(int64_t node_id, float& pos_estimate, float& vel_estimate)
{
  // FIX: previously this function returned immediately once the cache had
  // ANY entry for this node_id, so the RTR-request code below only ever
  // ran once (right at startup, while the cache was still empty). After
  // that first reply landed, every subsequent call kept returning that
  // same frozen value forever — read() never triggered a fresh request
  // again, even while the motor was actively moving. Fix: always send a
  // periodic re-request (every 20 calls, ~0.4s at 50Hz), then return
  // whatever is currently cached (last-known value) without blocking.
  static std::unordered_map<int64_t, int> request_counter;
  static std::mutex request_counter_mutex;
  bool should_request = false;
  {
    std::lock_guard<std::mutex> rc_lock(request_counter_mutex);
    should_request = (request_counter[node_id]++ % 20 == 0);
  }

  if (should_request) {
    uint32_t can_id = (static_cast<uint32_t>(node_id) << 5) | 0x009;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id | CAN_RTR_FLAG;
    frame.can_dlc = 0;

    std::lock_guard<std::mutex> lock(can_mutex_);
    canSend(frame);
  }

  std::lock_guard<std::mutex> cache_lock(cache_mutex_);
  if (encoder_pos_cache_.find(node_id) != encoder_pos_cache_.end() &&
      encoder_vel_cache_.find(node_id) != encoder_vel_cache_.end())
  {
    pos_estimate = encoder_pos_cache_[node_id];
    vel_estimate = encoder_vel_cache_[node_id];
    return true;
  }

  return false;
}

bool ODriveCAN::get_iq_measured(int64_t node_id, float& iq_measured, float& iq_setpoint)
{
  // Send a request at 2 Hz to avoid saturating the bus
  // FIX: same unsynchronized data race as get_encoder_estimates() — see
  // the comment there for why this matters.
  static std::unordered_map<int64_t, int> request_counter;
  static std::mutex request_counter_mutex;
  bool should_request = false;
  {
    std::lock_guard<std::mutex> rc_lock(request_counter_mutex);
    should_request = (request_counter[node_id]++ % 50 == 0);
  }

  if (should_request) {
    // CMD 0x014 = Get_Iq
    uint32_t can_id = (static_cast<uint32_t>(node_id) << 5) | 0x014;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id | CAN_RTR_FLAG;
    frame.can_dlc = 0;
    
    std::lock_guard<std::mutex> lock(can_mutex_);
    canSend(frame);
  }
  
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (iq_measured_cache_.find(node_id) != iq_measured_cache_.end()) {
    iq_measured = iq_measured_cache_[node_id];
    // iq_setpoint not implemented yet
    iq_setpoint = 0.0f;
    return true;
  }
  
  return false;
}

bool ODriveCAN::get_vbus_voltage(int64_t node_id, float& vbus_voltage)
{
  // Send a request very rarely (1 Hz instead of 100 Hz)
  // FIX: same unsynchronized data race as get_encoder_estimates() — see
  // the comment there for why this matters.
  static std::unordered_map<int64_t, int> request_counter;
  static std::mutex request_counter_mutex;
  bool should_request = false;
  {
    std::lock_guard<std::mutex> rc_lock(request_counter_mutex);
    should_request = (request_counter[node_id]++ % 100 == 0);
  }

  if (should_request) {
    // CMD 0x017 = Get_Vbus_Voltage
    uint32_t can_id = (static_cast<uint32_t>(node_id) << 5) | 0x017;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id | CAN_RTR_FLAG;
    frame.can_dlc = 0;
    
    std::lock_guard<std::mutex> lock(can_mutex_);
    canSend(frame);
  }
  
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (vbus_voltage_cache_.find(node_id) != vbus_voltage_cache_.end()) {
    vbus_voltage = vbus_voltage_cache_[node_id];
    return true;
  }
  
  return false;
}

bool ODriveCAN::get_heartbeat(int64_t node_id, uint32_t & axis_error, uint8_t & axis_state)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  const auto err_it = heartbeat_error_cache_.find(node_id);
  const auto state_it = heartbeat_state_cache_.find(node_id);
  if (err_it == heartbeat_error_cache_.end() || state_it == heartbeat_state_cache_.end()) {
    return false;
  }
  axis_error = err_it->second;
  axis_state = state_it->second;
  return true;
}

void ODriveCAN::invalidate_heartbeat(int64_t node_id)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  heartbeat_error_cache_.erase(node_id);
  heartbeat_state_cache_.erase(node_id);
}

bool ODriveCAN::wait_for_axis_state(
  int64_t node_id, uint8_t expected_state, uint32_t max_axis_error, int timeout_ms)
{
  uint64_t baseline_seq = 0;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    baseline_seq = heartbeat_seq_cache_[node_id];
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    uint32_t axis_error = 0;
    uint8_t axis_state = 0;
    uint64_t seq = 0;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      seq = heartbeat_seq_cache_[node_id];
      const auto err_it = heartbeat_error_cache_.find(node_id);
      const auto state_it = heartbeat_state_cache_.find(node_id);
      if (err_it != heartbeat_error_cache_.end() && state_it != heartbeat_state_cache_.end()) {
        axis_error = err_it->second;
        axis_state = state_it->second;
      } else {
        axis_error = UINT32_MAX;
        axis_state = 0;
      }
    }

    if (seq > baseline_seq &&
        axis_state == expected_state &&
        axis_error <= max_axis_error)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool ODriveCAN::wait_for_encoder_estimate(
  int64_t node_id, float & pos_estimate, float & vel_estimate, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (encoder_pos_cache_.find(node_id) != encoder_pos_cache_.end() &&
          encoder_vel_cache_.find(node_id) != encoder_vel_cache_.end())
      {
        pos_estimate = encoder_pos_cache_[node_id];
        vel_estimate = encoder_vel_cache_[node_id];
        return true;
      }
    }

    uint32_t can_id = (static_cast<uint32_t>(node_id) << 5) | 0x009;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id | CAN_RTR_FLAG;
    frame.can_dlc = 0;
    {
      std::lock_guard<std::mutex> lock(can_mutex_);
      canSend(frame);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Private functions
int ODriveCAN::canSend(const struct can_frame& frame)
{
  ssize_t ret = ::write(can_socket_, &frame, sizeof(frame));
  if (ret < 0) {
    if (errno == ENOBUFS) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ODriveCAN"), steady_clock, 2000,
        "CAN TX queue full (ENOBUFS). Run: sudo ip link set can0 txqueuelen 1000");
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("ODriveCAN"), "Error writing to CAN socket: %s",
        std::strerror(errno));
    }
    return -1;
  }
  if (static_cast<size_t>(ret) != sizeof(frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"),
                 "Partial write to CAN socket: %zd bytes (expected %zu)",
                 ret, static_cast<size_t>(sizeof(frame)));
    return -1;
  }
  return 0;
}

int ODriveCAN::canReceive(struct can_frame& frame, int timeout_ms)
{
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(can_socket_, &readfds);
  
  int ret = select(can_socket_ + 1, &readfds, NULL, NULL, &tv);
  if (ret > 0) {
    if (FD_ISSET(can_socket_, &readfds)) {
      // Use ::read to avoid colliding with class template read(...)
      ssize_t r = ::read(can_socket_, &frame, sizeof(frame));
      if (r < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error reading from CAN socket: %s", std::strerror(errno));
        return -1;
      }
      if (static_cast<size_t>(r) != sizeof(frame)) {
        // Unexpected size; still return error
        RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Unexpected CAN frame size: %zd", r);
        return -1;
      }
      return 0;
    }
  } else if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "select() error on CAN socket: %s", std::strerror(errno));
    return -1;
  }

  // Timeout or no data
  return -1;
}

uint32_t ODriveCAN::endpoint_to_can_id(int64_t node_id, short endpoint_id, bool is_request)
{
  // Basic endpoint-to-CAN-ID conversion implementation
  // Adjust according to your ODrive's CAN protocol
  // This code preserves the current logic: node_id in high bits, endpoint low bits
  return static_cast<uint32_t>((static_cast<uint32_t>(node_id) << 5) | (static_cast<uint32_t>(endpoint_id) & 0x1F));
}

bool ODriveCAN::send_can_message(int64_t node_id, uint32_t command_id, const uint8_t* data, uint8_t data_len)
{
  // Classic CAN max 8 bytes payload
  constexpr size_t ODRIVE_LOCAL_CAN_MAX_DLEN = 8;
  if (static_cast<size_t>(data_len) > ODRIVE_LOCAL_CAN_MAX_DLEN) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"),
                 "send_can_message: data_len %zu > %zu (not supported).",
                 static_cast<size_t>(data_len),
                 static_cast<size_t>(ODRIVE_LOCAL_CAN_MAX_DLEN));
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = static_cast<uint32_t>((static_cast<uint32_t>(node_id) << 5) | (command_id & 0x1F));
  frame.can_dlc = data_len;
  if (data_len > 0) {
    std::memcpy(frame.data, data, data_len);
  }
  
  return canSend(frame) == 0;
}

bool ODriveCAN::process_can_message(const struct can_frame& frame)
{
  int64_t node_id = static_cast<int64_t>(frame.can_id >> 5);
  uint32_t command_id = static_cast<uint32_t>(frame.can_id & 0x1F);
  
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  switch (command_id) {
    case 0x001:  // Heartbeat
      if (frame.can_dlc >= 5) {
        heartbeat_error_cache_[node_id] = bytes_to_int32(frame.data);
        heartbeat_state_cache_[node_id] = frame.data[4];
        heartbeat_seq_cache_[node_id]++;
      }
      break;

    case 0x009: // Encoder estimates
      if (frame.can_dlc >= 8) {
        encoder_pos_cache_[node_id] = bytes_to_float(frame.data);
        encoder_vel_cache_[node_id] = bytes_to_float(frame.data + 4);
      }
      break;
      
    case 0x014: // IQ measured
      if (frame.can_dlc >= 4) {
        iq_measured_cache_[node_id] = bytes_to_float(frame.data);
      }
      break;
      
    case 0x017: // VBUS voltage
      if (frame.can_dlc >= 4) {
        vbus_voltage_cache_[node_id] = bytes_to_float(frame.data);
      }
      break;
      
    default:
      return false;
  }
  
  return true;
}

// Helper functions
void ODriveCAN::float_to_bytes(float f, uint8_t* bytes)
{
  std::memcpy(bytes, &f, sizeof(float));
}

float ODriveCAN::bytes_to_float(const uint8_t* bytes)
{
  float f;
  std::memcpy(&f, bytes, sizeof(float));
  return f;
}

void ODriveCAN::int32_to_bytes(int32_t i, uint8_t* bytes)
{
  std::memcpy(bytes, &i, sizeof(int32_t));
}

int32_t ODriveCAN::bytes_to_int32(const uint8_t* bytes)
{
  int32_t i;
  std::memcpy(&i, bytes, sizeof(int32_t));
  return i;
}

// CAN receive thread
void ODriveCAN::receive_loop()
{
  RCLCPP_INFO(rclcpp::get_logger("ODriveCAN"), "CAN receive loop started");
  
  struct can_frame frame;
  
  while (running_) {
    std::memset(&frame, 0, sizeof(frame));
    
    // Non-blocking read with short timeout
    if (canReceive(frame, 10) == 0) {
      // Process the received message
      process_can_message(frame);
    }
    
    // Small pause to avoid overloading the CPU
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ODriveCAN"), "CAN receive loop stopped");
}

// Instanciations explicites des templates
template int ODriveCAN::read<int32_t>(int64_t, short, int32_t&);
template int ODriveCAN::read<float>(int64_t, short, float&);
template int ODriveCAN::read<bool>(int64_t, short, bool&);
template int ODriveCAN::read<uint32_t>(int64_t, short, uint32_t&);
template int ODriveCAN::read<uint64_t>(int64_t, short, uint64_t&);
template int ODriveCAN::read<uint16_t>(int64_t, short, uint16_t&);
template int ODriveCAN::read<uint8_t>(int64_t, short, uint8_t&);
template int ODriveCAN::read<int16_t>(int64_t, short, int16_t&);

template int ODriveCAN::write<int32_t>(int64_t, short, const int32_t&);
template int ODriveCAN::write<float>(int64_t, short, const float&);
template int ODriveCAN::write<bool>(int64_t, short, const bool&);
template int ODriveCAN::write<uint32_t>(int64_t, short, const uint32_t&);

}  // namespace odrive