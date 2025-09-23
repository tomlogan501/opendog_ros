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

#include "odrive_can.hpp"
#include <chrono>
#include <thread>

namespace odrive
{

ODriveCAN::ODriveCAN() : can_socket_(-1) {}

ODriveCAN::~ODriveCAN()
{
  if (can_socket_ >= 0) {
    close(can_socket_);
  }
}

int ODriveCAN::init(const std::vector<std::vector<int64_t>>& node_ids, const std::string& can_interface)
{
  can_interface_ = can_interface;

  // Create socket
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error while opening socket");
    return -1;
  }

  // Get interface index
  struct ifreq ifr;
  strcpy(ifr.ifr_name, can_interface_.c_str());
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error getting interface index for %s", can_interface_.c_str());
    close(can_socket_);
    can_socket_ = -1;
    return -1;
  }

  // Bind socket to interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error binding socket to interface %s", can_interface_.c_str());
    close(can_socket_);
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

  return 0;
}

// Implémentation des templates de lecture/écriture pour compatibilité avec l'interface existante
template <typename T>
int ODriveCAN::read(int64_t node_id, short endpoint_id, T& value)
{
  // Pour la compatibilité avec le code existant, nous implémentons une lecture générique
  // Cette méthode est moins efficace que les méthodes spécifiques
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  // Envoi de la commande de lecture
  uint32_t can_id = endpoint_to_can_id(node_id, endpoint_id, true);
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = 0; // Lecture sans données
  
  if (canSend(frame) != 0) {
    return -1;
  }
  
  // Attente de la réponse (simplifié)
  struct can_frame response;
  if (canReceive(response, 100) == 0) {
    // Traitement de la réponse basique
    if (sizeof(T) <= response.can_dlc) {
      std::memcpy(&value, response.data, sizeof(T));
      return 0;
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
  frame.can_id = can_id;
  frame.can_dlc = sizeof(T);
  std::memcpy(frame.data, &value, sizeof(T));
  
  return canSend(frame);
}

int ODriveCAN::call(int64_t node_id, short endpoint_id)
{
  // Pour un call, on envoie juste la commande sans données
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  uint32_t can_id = endpoint_to_can_id(node_id, endpoint_id, false);
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = 0;
  
  return canSend(frame);
}

// Implémentation des fonctions CAN spécifiques
bool ODriveCAN::send_set_axis_state(int64_t node_id, int32_t requested_state)
{
  return write(node_id, AXIS__REQUESTED_STATE, requested_state) == 0;
}

bool ODriveCAN::send_set_controller_mode(int64_t node_id, int32_t control_mode, int32_t input_mode)
{
  // Pack control_mode et input_mode dans un message
  uint32_t can_id = (node_id << 5) | 0x001; // Exemple d'ID CAN
  uint8_t data[8];
  int32_to_bytes(control_mode, data);
  int32_to_bytes(input_mode, data + 4);
  
  return send_can_message(node_id, 0x001, data, 8);
}

bool ODriveCAN::send_set_input_pos(int64_t node_id, float position, float velocity_feedforward, float torque_feedforward)
{
  uint8_t data[12];
  float_to_bytes(position, data);
  float_to_bytes(velocity_feedforward, data + 4);
  float_to_bytes(torque_feedforward, data + 8);
  
  return send_can_message(node_id, 0x00C, data, 12);
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
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (encoder_pos_cache_.find(node_id) != encoder_pos_cache_.end() &&
      encoder_vel_cache_.find(node_id) != encoder_vel_cache_.end()) {
    pos_estimate = encoder_pos_cache_[node_id];
    vel_estimate = encoder_vel_cache_[node_id];
    return true;
  }
  
  return false;
}

bool ODriveCAN::get_iq_measured(int64_t node_id, float& iq_measured, float& iq_setpoint)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (iq_measured_cache_.find(node_id) != iq_measured_cache_.end()) {
    iq_measured = iq_measured_cache_[node_id];
    // iq_setpoint non implémenté pour le moment
    iq_setpoint = 0.0f;
    return true;
  }
  
  return false;
}

bool ODriveCAN::get_vbus_voltage(int64_t node_id, float& vbus_voltage)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (vbus_voltage_cache_.find(node_id) != vbus_voltage_cache_.end()) {
    vbus_voltage = vbus_voltage_cache_[node_id];
    return true;
  }
  
  return false;
}

// Fonctions privées
int ODriveCAN::canSend(const struct can_frame& frame)
{
  int ret = write(can_socket_, &frame, sizeof(frame));
  if (ret != sizeof(frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveCAN"), "Error writing to CAN socket: %s", strerror(errno));
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
      ret = read(can_socket_, &frame, sizeof(frame));
      if (ret == sizeof(frame)) {
        return 0;
      }
    }
  }
  
  return -1;
}

uint32_t ODriveCAN::endpoint_to_can_id(int64_t node_id, short endpoint_id, bool is_request)
{
  // Implémentation basique de conversion endpoint vers CAN ID
  // À adapter selon le protocole CAN de votre ODrive
  return (node_id << 5) | (endpoint_id & 0x1F);
}

bool ODriveCAN::send_can_message(int64_t node_id, uint32_t command_id, const uint8_t* data, uint8_t data_len)
{
  struct can_frame frame;
  frame.can_id = (node_id << 5) | command_id;
  frame.can_dlc = data_len;
  std::memcpy(frame.data, data, data_len);
  
  return canSend(frame) == 0;
}

bool ODriveCAN::process_can_message(const struct can_frame& frame)
{
  int64_t node_id = frame.can_id >> 5;
  uint32_t command_id = frame.can_id & 0x1F;
  
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  switch (command_id) {
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

// Instanciations explicites des templates
template int ODriveCAN::read<int32_t>(int64_t, short, int32_t&);
template int ODriveCAN::read<float>(int64_t, short, float&);
template int ODriveCAN::read<bool>(int64_t, short, bool&);
template int ODriveCAN::read<uint32_t>(int64_t, short, uint32_t&);
template int ODriveCAN::read<uint64_t>(int64_t, short, uint64_t&);

template int ODriveCAN::write<int32_t>(int64_t, short, const int32_t&);
template int ODriveCAN::write<float>(int64_t, short, const float&);
template int ODriveCAN::write<bool>(int64_t, short, const bool&);
template int ODriveCAN::write<uint32_t>(int64_t, short, const uint32_t&);

}  // namespace odrive
