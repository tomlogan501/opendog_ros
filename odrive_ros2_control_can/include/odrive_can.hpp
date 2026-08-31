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

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <atomic>
#include "odrive_endpoints.hpp"


#include "rclcpp/rclcpp.hpp"

namespace odrive
{

class ODriveCAN
{
public:
  ODriveCAN();
  ~ODriveCAN();

  int init(const std::vector<std::vector<int64_t>>& node_ids, const std::string& can_interface = "can0");
  
  template <typename T>
  int read(int64_t node_id, short endpoint_id, T& value);
  
  template <typename T>
  int write(int64_t node_id, short endpoint_id, const T& value);
  
  int call(int64_t node_id, short endpoint_id);

  // CAN-specific functions for better performance
  bool send_heartbeat_command(int64_t node_id);
  bool send_encoder_estimates_request(int64_t node_id);
  bool send_set_input_pos(int64_t node_id, float position, float velocity_feedforward = 0.0f, float torque_feedforward = 0.0f);
  bool send_set_input_vel(int64_t node_id, float velocity, float torque_feedforward = 0.0f);
  bool send_set_input_torque(int64_t node_id, float torque);
  bool send_set_controller_mode(int64_t node_id, int32_t control_mode, int32_t input_mode = 0);
  bool send_set_axis_state(int64_t node_id, int32_t requested_state);
  bool send_clear_errors(int64_t node_id);
  bool send_set_limits(int64_t node_id, float velocity_limit, float current_limit);

  // Reading CAN data
  bool get_encoder_estimates(int64_t node_id, float& pos_estimate, float& vel_estimate);
  bool get_iq_measured(int64_t node_id, float& iq_measured, float& iq_setpoint);
  bool get_vbus_voltage(int64_t node_id, float& vbus_voltage);
  bool get_heartbeat(int64_t node_id, uint32_t & axis_error, uint8_t & axis_state);

  // Blocking helpers used during axis activation
  bool wait_for_encoder_estimate(
    int64_t node_id, float & pos_estimate, float & vel_estimate, int timeout_ms = 500);
  bool wait_for_axis_state(
    int64_t node_id, uint8_t expected_state, uint32_t max_axis_error = 0, int timeout_ms = 500);

  void invalidate_heartbeat(int64_t node_id);

private:
  int can_socket_;
  std::string can_interface_;
  std::map<int64_t, int64_t> node_id_map_;
  std::mutex can_mutex_;

  // Cache for frequently read data
  std::unordered_map<int64_t, float> encoder_pos_cache_;
  std::unordered_map<int64_t, float> encoder_vel_cache_;
  std::unordered_map<int64_t, float> iq_measured_cache_;
  std::unordered_map<int64_t, float> vbus_voltage_cache_;
  std::unordered_map<int64_t, uint32_t> heartbeat_error_cache_;
  std::unordered_map<int64_t, uint8_t> heartbeat_state_cache_;
  std::unordered_map<int64_t, uint64_t> heartbeat_seq_cache_;
  std::mutex cache_mutex_;

  // CAN receive thread
  std::thread receive_thread_;
  std::atomic<bool> running_;
  void receive_loop();

  int canSend(const struct can_frame& frame);
  int canReceive(struct can_frame& frame, int timeout_ms = 10);
  
  // Convert ODrive endpoint to CAN command
  uint32_t endpoint_to_can_id(int64_t node_id, short endpoint_id, bool is_request = false);
  
  // Handling ODrive-specific CAN messages
  bool process_can_message(const struct can_frame& frame);
  
  // Send raw CAN message
  bool send_can_message(int64_t node_id, uint32_t command_id, const uint8_t* data, uint8_t data_len);
  
  // Wait for CAN response
  bool wait_for_can_response(int64_t node_id, uint32_t expected_command, int timeout_ms = 100);

  // Helper functions pour pack/unpack data
  void float_to_bytes(float f, uint8_t* bytes);
  float bytes_to_float(const uint8_t* bytes);
  void int32_to_bytes(int32_t i, uint8_t* bytes);
  int32_t bytes_to_int32(const uint8_t* bytes);
};

}  // namespace odrive