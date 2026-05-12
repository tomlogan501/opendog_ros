#include "odrive_hardware_layer/hardware_layer.hpp"
#include "odrive_hardware_layer/odrive_can_protocol.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <thread>

namespace odrive_hardware_layer
{

// ─────────────────────────────────────────────────────────
// on_init — called once at startup
// Reads joint config from URDF parameters
// ─────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdriveHardwareLayer::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read CAN interface name from URDF params (default: can0)
  can_interface_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface")
    : "can0";

  n_joints_ = info_.joints.size();

  // Resize state and command vectors
  hw_positions_.resize(n_joints_, 0.0);
  hw_velocities_.resize(n_joints_, 0.0);
  hw_efforts_.resize(n_joints_, 0.0);
  hw_commands_positions_.resize(n_joints_, 0.0);
  hw_commands_velocities_.resize(n_joints_, 0.0);
  hw_commands_efforts_.resize(n_joints_, 0.0);

  // Read node IDs from URDF joint parameters
  node_ids_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; i++) {
    node_ids_[i] = std::stoi(
      info_.joints[i].parameters.at("node_id"));

    RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
      "Joint %s → node_id %d",
      info_.joints[i].name.c_str(), node_ids_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
// on_activate — called when robot activates
// Opens CAN socket, clears errors, calibrates motors
// ─────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdriveHardwareLayer::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "Activating... opening CAN socket on %s", can_interface_.c_str());

  // Open SocketCAN socket
  can_sock_ = can_open(can_interface_);
  if (can_sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveHardwareLayer"),
      "Failed to open CAN socket on %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "CAN socket opened. Waiting for ODrive heartbeats...");

  // Wait for heartbeat from each motor and clear errors
  for (size_t i = 0; i < n_joints_; i++) {
    uint32_t axis_error;
    uint8_t  axis_state;

    if (!get_heartbeat(can_sock_, node_ids_[i], axis_error, axis_state, 5000)) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveHardwareLayer"),
        "No heartbeat from node %d", node_ids_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
      "Node %d alive. State=%d Error=0x%X",
      node_ids_[i], axis_state, axis_error);

    clear_errors(can_sock_, node_ids_[i]);
  }

  // Calibrate all motors
  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "Calibrating all motors...");

  for (size_t i = 0; i < n_joints_; i++) {
    set_axis_state(can_sock_, node_ids_[i], AXIS_STATE_MOTOR_CALIBRATION);
  }

  std::this_thread::sleep_for(std::chrono::seconds(8));

  // Enter closed loop control on all motors
  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "Entering closed loop control...");

  for (size_t i = 0; i < n_joints_; i++) {
    clear_errors(can_sock_, node_ids_[i]);
    set_control_mode(can_sock_, node_ids_[i],
      CONTROL_MODE_POSITION_CONTROL,
      INPUT_MODE_TRAP_TRAJ);
    set_axis_state(can_sock_, node_ids_[i], AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "All motors active. Hardware interface ready.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
// on_deactivate — called when robot shuts down
// Sets all motors to IDLE and closes CAN socket
// ─────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdriveHardwareLayer::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "Deactivating... setting all motors to IDLE");

  for (size_t i = 0; i < n_joints_; i++) {
    set_input_velocity(can_sock_, node_ids_[i], 0.0f);
    set_axis_state(can_sock_, node_ids_[i], AXIS_STATE_IDLE);
  }

  can_close(can_sock_);

  RCLCPP_INFO(rclcpp::get_logger("OdriveHardwareLayer"),
    "CAN socket closed. Hardware interface deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
// export_state_interfaces
// Tells ros2_control what data we can provide
// ─────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
OdriveHardwareLayer::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < n_joints_; i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_efforts_[i]);
  }

  return state_interfaces;
}

// ─────────────────────────────────────────────────────────
// export_command_interfaces
// Tells ros2_control what commands we accept
// ─────────────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
OdriveHardwareLayer::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < n_joints_; i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_positions_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_velocities_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_commands_efforts_[i]);
  }

  return command_interfaces;
}

// ─────────────────────────────────────────────────────────
// read — called every control loop (1000Hz)
// Reads position + velocity from each ODrive over CAN
// Converts turns → radians for ROS2
// ─────────────────────────────────────────────────────────

hardware_interface::return_type OdriveHardwareLayer::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < n_joints_; i++) {
    float pos, vel;
    if (get_encoder_estimates(can_sock_, node_ids_[i], pos, vel, 10)) {
      // ODrive returns turns — convert to radians for ROS2
      hw_positions_[i]  = pos * 2.0 * M_PI;
      hw_velocities_[i] = vel * 2.0 * M_PI;
    }
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────
// write — called every control loop (1000Hz)
// Sends position commands to each ODrive over CAN
// Converts radians → turns for ODrive
// ─────────────────────────────────────────────────────────

hardware_interface::return_type OdriveHardwareLayer::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < n_joints_; i++) {
    // ROS2 uses radians — convert to turns for ODrive
    float pos = hw_commands_positions_[i] / (2.0 * M_PI);
    float vel = hw_commands_velocities_[i] / (2.0 * M_PI);

    set_input_position(can_sock_, node_ids_[i], pos,
      static_cast<int16_t>(vel * 1000), 0);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace odrive_hardware_layer

// Register plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_layer::OdriveHardwareLayer,
  hardware_interface::SystemInterface)