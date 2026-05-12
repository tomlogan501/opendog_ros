#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace odrive_hardware_layer
{

class OdriveHardwareLayer : public hardware_interface::SystemInterface
{
public:

  // ── Lifecycle callbacks ──────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface exports ────────────────────────────────
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // ── Control loop ────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:

  // CAN socket file descriptor
  int can_sock_{-1};

  // CAN interface name (e.g. "can0")
  std::string can_interface_;

  // Number of joints
  size_t n_joints_{0};

  // Node IDs for each joint (set in URDF)
  std::vector<int> node_ids_;

  // State interfaces — what we READ from hardware
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Command interfaces — what we WRITE to hardware
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
};

}  // namespace odrive_hardware_layer