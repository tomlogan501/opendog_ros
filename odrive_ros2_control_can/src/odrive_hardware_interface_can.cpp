// Copyright 2021 Factor Robotics
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

#include "odrive_hardware_interface_can.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "odrive_endpoints.hpp"
#include <set>
#include <algorithm>

namespace odrive_ros2_control
{
CallbackReturn ODriveHardwareInterfaceCAN::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Initializing ODrive CAN Hardware Interface");

  // Utilisation d'un set pour stocker les node_ids uniques
  std::set<int> unique_node_ids;

  // Initialisation des vecteurs de données
  hw_vbus_voltages_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Vecteurs pour stocker les node_ids par joint/sensor (on garde pour compatibilité)
  joint_node_ids_.resize(info_.joints.size());
  sensor_node_ids_.resize(info_.sensors.size());

  // Lecture des node_ids pour les sensors
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    int node_id = std::stoi(info_.sensors[i].parameters.at("node_id"));
    sensor_node_ids_[i] = node_id;
    unique_node_ids.insert(node_id);
    
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
      "Sensor '%s' -> node_id: %d", 
      info_.sensors[i].name.c_str(), 
      node_id
    );
  }

  // Lecture des paramètres pour les joints
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = std::stoi(info_.joints[i].parameters.at("node_id"));
    joint_node_ids_[i] = node_id;
    unique_node_ids.insert(node_id);
    
    axes_.emplace_back(std::stoi(info_.joints[i].parameters.at("axis")));
    enable_watchdogs_.emplace_back(info_.joints[i].parameters.at("enable_watchdog") == "true");

    RCLCPP_INFO(
      rclcpp::get_logger("ODriveHardwareInterfaceCAN"),
      "Joint '%s' -> node_id: %d, axis: %d, watchdog: %s",
      info_.joints[i].name.c_str(), 
      node_id, 
      axes_.back(),
      enable_watchdogs_.back() ? "enabled" : "disabled"
    );
  }

  // Afficher tous les node_ids uniques détectés
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"),
    "Detected %zu unique ODrive node IDs", 
    unique_node_ids.size()
  );

  // Convertir le set en vector pour l'initialisation de ODriveCAN
  std::vector<std::vector<int64_t>> node_ids_grouped(2);
  
  // Groupe 0: sensors
  for (int node_id : sensor_node_ids_) {
    if (std::find(node_ids_grouped[0].begin(), node_ids_grouped[0].end(), node_id) == node_ids_grouped[0].end()) {
      node_ids_grouped[0].push_back(static_cast<int64_t>(node_id));
    }
  }
  
  // Groupe 1: joints
  for (int node_id : joint_node_ids_) {
    if (std::find(node_ids_grouped[1].begin(), node_ids_grouped[1].end(), node_id) == node_ids_grouped[1].end()) {
      node_ids_grouped[1].push_back(static_cast<int64_t>(node_id));
    }
  }

  // Initialisation de l'interface CAN
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Initializing ODriveCAN");
  odrive_can_ = new ODriveCAN();

  CHECK_TS(odrive_can_->init(node_ids_grouped));
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "ODriveCAN initialized successfully");

  // Configuration des constantes de couple et watchdogs
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = joint_node_ids_[i];
    float torque_constant;

    // Lecture de la constante de couple
    if (odrive_can_->read(node_id, AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + per_axis_offset * axes_[i], torque_constant) == 0) {
      torque_constants_.emplace_back(torque_constant);
      RCLCPP_INFO(
        rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
        "Node %d, Axis %d: torque constant = %.4f Nm/A", 
        node_id, axes_[i], torque_constant
      );
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
        "Failed to read torque constant for node %d axis %d, using default 0.1", 
        node_id, axes_[i]
      );
      torque_constants_.emplace_back(0.1f);
    }

    // Configuration watchdog si activé
    if (enable_watchdogs_[i]) {
      float timeout = std::stof(info_.joints[i].parameters.at("watchdog_timeout"));
      odrive_can_->write(node_id, AXIS__CONFIG__WATCHDOG_TIMEOUT + per_axis_offset * axes_[i], timeout);
      odrive_can_->write(node_id, AXIS__CONFIG__ENABLE_WATCHDOG + per_axis_offset * axes_[i], true);
      RCLCPP_INFO(
        rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
        "Node %d, Axis %d: watchdog enabled with timeout %.3f seconds", 
        node_id, axes_[i], timeout
      );
    }
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
    "ODrive CAN Hardware Interface initialized with %zu joints and %zu sensors", 
    info_.joints.size(),
    info_.sensors.size()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterfaceCAN::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Activating ODrive CAN Hardware Interface");

  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = joint_node_ids_[i];

    // Alimentation du watchdog si activé
    if (enable_watchdogs_[i]) {
      odrive_can_->call(node_id, AXIS__WATCHDOG_FEED + per_axis_offset * axes_[i]);
      RCLCPP_DEBUG(
        rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
        "Node %d, Axis %d: watchdog fed", 
        node_id, axes_[i]
      );
    }

    // Effacement des erreurs
    odrive_can_->call(node_id, CLEAR_ERRORS);

    RCLCPP_INFO(
      rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
      "Node %d, Axis %d: activated and errors cleared", 
      node_id, axes_[i]
    );
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "ODrive CAN Hardware Interface activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterfaceCAN::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "Deactivating ODrive CAN Hardware Interface");

  // MODE READ ONLY : Ne pas envoyer de commandes IDLE aux moteurs
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
    "Mode READ ONLY: Motors left in current state (not set to IDLE)"
  );
  
  /* DÉSACTIVÉ POUR MODE READ ONLY
  int32_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = joint_node_ids_[i];
    odrive_can_->write(node_id, AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state);
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
      "Node %d, Axis %d: set to IDLE state", 
      node_id, axes_[i]
    );
  }
  */

  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterfaceCAN"), "ODrive CAN Hardware Interface deactivated");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterfaceCAN::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
    "Exporting state interfaces for %zu sensors and %zu joints",
    info_.sensors.size(), 
    info_.joints.size()
  );

  // Interfaces pour les sensors (vbus voltage)
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
  }

  // Interfaces pour les joints
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));

    // Interfaces pour les données de diagnostic
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveHardwareInterfaceCAN::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
    "Exporting command interfaces for %zu joints", 
    info_.joints.size()
  );

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

return_type ODriveHardwareInterfaceCAN::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const std::string& key : stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        control_level_[i] = integration_level_t::UNDEFINED;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Joint '%s' control level reset to UNDEFINED", 
          info_.joints[i].name.c_str()
        );
      }
    }
  }

  for (const std::string& key : start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        control_level_[i] = integration_level_t::POSITION;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Joint '%s' control level set to POSITION", 
          info_.joints[i].name.c_str()
        );
      } else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        control_level_[i] = integration_level_t::VELOCITY;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Joint '%s' control level set to VELOCITY", 
          info_.joints[i].name.c_str()
        );
      } else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        control_level_[i] = integration_level_t::EFFORT;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Joint '%s' control level set to EFFORT", 
          info_.joints[i].name.c_str()
        );
      }
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterfaceCAN::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  // MODE READ ONLY : Pas d'envoi de commandes aux moteurs
  // Les ODrives doivent être configurés manuellement en CLOSED_LOOP_CONTROL
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
    "Mode READ ONLY: No commands sent to motors"
  );
  
  /* DÉSACTIVÉ POUR MODE READ ONLY
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = joint_node_ids_[i];
    int32_t requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;

    // Configuration du mode de contrôle selon le niveau d'intégration
    switch (control_level_[i]) {
      case integration_level_t::POSITION:
        odrive_can_->send_set_controller_mode(node_id, 3, 1); // Position control
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        RCLCPP_INFO(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Node %d, Axis %d: Position control mode activated", 
          node_id, axes_[i]
        );
        break;

      case integration_level_t::VELOCITY:
        odrive_can_->send_set_controller_mode(node_id, 2, 1); // Velocity control
        hw_commands_velocities_[i] = hw_velocities_[i];
        hw_commands_efforts_[i] = 0;
        RCLCPP_INFO(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Node %d, Axis %d: Velocity control mode activated", 
          node_id, axes_[i]
        );
        break;

      case integration_level_t::EFFORT:
        odrive_can_->send_set_controller_mode(node_id, 1, 1); // Torque control
        hw_commands_efforts_[i] = hw_efforts_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Node %d, Axis %d: Torque control mode activated", 
          node_id, axes_[i]
        );
        break;

      case integration_level_t::UNDEFINED:
        requested_state = AXIS_STATE_IDLE;
        RCLCPP_INFO(
          rclcpp::get_logger("ODriveHardwareInterfaceCAN"), 
          "Node %d, Axis %d: Set to IDLE state", 
          node_id, axes_[i]
        );
        break;
    }

    // Changement d'état de l'axe
    odrive_can_->send_set_axis_state(node_id, requested_state);
  }
  */
  
  return return_type::OK;
}

return_type ODriveHardwareInterfaceCAN::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Compteur pour réduire la fréquence de lecture des données non-critiques
  static int slow_read_counter = 0;
  slow_read_counter++;
  
  // Lecture des données des sensors (vbus voltage) - seulement tous les 100 cycles (1 Hz)
  if (slow_read_counter % 100 == 0) {
    for (size_t i = 0; i < info_.sensors.size(); i++) {
      int node_id = sensor_node_ids_[i];
      float vbus_voltage;

      if (odrive_can_->get_vbus_voltage(node_id, vbus_voltage)) {
        hw_vbus_voltages_[i] = vbus_voltage;
      }
    }
  }

  // Lecture des données des joints - UNIQUEMENT position, vitesse et couple
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int node_id = joint_node_ids_[i];
    float pos_estimate, vel_estimate, iq_measured;

    // Lecture position et vitesse via CAN optimisé (messages heartbeat automatiques)
    if (odrive_can_->get_encoder_estimates(node_id, pos_estimate, vel_estimate)) {
      hw_positions_[i] = pos_estimate * 2 * M_PI;  // Conversion rev → rad
      hw_velocities_[i] = vel_estimate * 2 * M_PI; // Conversion rev/s → rad/s
    }

    // MODE READ ONLY : Lecture du couple désactivée pour éviter saturation CAN
    /* DÉSACTIVÉ TEMPORAIREMENT
    float iq_setpoint;
    if (odrive_can_->get_iq_measured(node_id, iq_measured, iq_setpoint)) {
      hw_efforts_[i] = iq_measured * torque_constants_[i];
    }
    */

    // Lecture des erreurs et températures - seulement tous les 10 cycles (10 Hz)
    if (slow_read_counter % 10 == 0) {
      uint32_t axis_error;
      uint64_t motor_error;
      uint16_t encoder_error;
      uint8_t controller_error;

      if (odrive_can_->read(node_id, AXIS__ERROR + per_axis_offset * axes_[i], axis_error) == 0) {
        hw_axis_errors_[i] = static_cast<double>(axis_error);
      }

      if (odrive_can_->read(node_id, AXIS__MOTOR__ERROR + per_axis_offset * axes_[i], motor_error) == 0) {
        hw_motor_errors_[i] = static_cast<double>(motor_error);
      }

      if (odrive_can_->read(node_id, AXIS__ENCODER__ERROR + per_axis_offset * axes_[i], encoder_error) == 0) {
        hw_encoder_errors_[i] = static_cast<double>(encoder_error);
      }

      if (odrive_can_->read(node_id, AXIS__CONTROLLER__ERROR + per_axis_offset * axes_[i], controller_error) == 0) {
        hw_controller_errors_[i] = static_cast<double>(controller_error);
      }
    }

    // Lecture des températures - seulement tous les 100 cycles (1 Hz)
    if (slow_read_counter % 100 == 0) {
      float fet_temperature, motor_temperature;

      if (odrive_can_->read(node_id, AXIS__MOTOR__FET_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i], fet_temperature) == 0) {
        hw_fet_temperatures_[i] = fet_temperature;
      }

      if (odrive_can_->read(node_id, AXIS__MOTOR__MOTOR_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i], motor_temperature) == 0) {
        hw_motor_temperatures_[i] = motor_temperature;
      }
    }
  }
  
  return return_type::OK;
}

return_type ODriveHardwareInterfaceCAN::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // La méthode write reste vide car les commandes sont envoyées via perform_command_mode_switch
  // et les méthodes de ODriveCAN (send_set_controller_mode, send_set_axis_state, etc.)
  return return_type::OK;
}

}  // namespace odrive_ros2_control

PLUGINLIB_EXPORT_CLASS(
  odrive_ros2_control::ODriveHardwareInterfaceCAN,
  hardware_interface::SystemInterface)
