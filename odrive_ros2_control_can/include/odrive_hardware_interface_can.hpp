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

#include <cmath>
#include <set>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "odrive_can.hpp"
#include "odrive_endpoints.hpp"
#include "visibility_control.hpp"

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CHECK_TS(status)                                                                   \
  do {                                                                                     \
    int ret = (status);                                                                    \
    if (ret != 0) {                                                                        \
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Error: %d", ret);   \
      return CallbackReturn::ERROR;                                                        \
    }                                                                                      \
  } while (0)

#define CHECK_RW(status)                                                                   \
  do {                                                                                     \
    int ret = (status);                                                                    \
    if (ret != 0) {                                                                        \
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Error: %d", ret);   \
      return return_type::ERROR;                                                           \
    }                                                                                      \
  } while (0)

using namespace odrive;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace odrive_ros2_control
{
class ODriveHardwareInterfaceCAN : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterfaceCAN)

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  HARDWARE_INTERFACE_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  HARDWARE_INTERFACE_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  HARDWARE_INTERFACE_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool idle_and_clear_all_axes();
  bool is_axis_in_healthy_closed_loop(size_t joint_index);
  bool is_axis_clean_idle(size_t joint_index);
  void sync_command_from_encoder(size_t joint_index);
  bool enter_position_closed_loop(size_t joint_index);
  bool verify_closed_loop_stable(size_t joint_index, int hold_ms);
  // Interface ODriveCAN
  ODriveCAN * odrive_can_;

  // Node IDs uniques (set pour éviter les doublons)
  std::set<int> unique_node_ids_;
  
  // Node IDs par joint/sensor (vecteurs pour accès indexé)
  std::vector<int> joint_node_ids_;
  std::vector<int> sensor_node_ids_;

  // Configuration des axes
  std::vector<int> axes_;
  std::vector<float> torque_constants_;
  std::vector<double> gear_ratios_;
  std::vector<double> zero_offsets_;    
  std::vector<bool> enable_watchdogs_;

  // État des sensors (vbus voltage)
  std::vector<double> hw_vbus_voltages_;

  // Commandes des joints
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;

  // États des joints
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Diagnostics des joints
  std::vector<double> hw_axis_errors_;
  std::vector<double> hw_motor_errors_;
  std::vector<double> hw_encoder_errors_;
  std::vector<double> hw_controller_errors_;
  std::vector<double> hw_fet_temperatures_;
  std::vector<double> hw_motor_temperatures_;

  // Niveaux d'intégration (modes de contrôle)
  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };

  std::vector<integration_level_t> control_level_;
  std::vector<float> last_sent_position_turns_;
};

}  // namespace odrive_ros2_control
