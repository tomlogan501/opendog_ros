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

#include "odrive_hardware_interface/odrive_usb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <set>

namespace odrive
{
ODriveUSB::ODriveUSB() { libusb_context_ = NULL; }

ODriveUSB::~ODriveUSB()
{
  for (auto it = odrive_map_.begin(); it != odrive_map_.end(); it++) {
    libusb_release_interface(it->second, 2);
    libusb_close(it->second);
  }
  odrive_map_.clear();

  if (libusb_context_) {
    libusb_exit(libusb_context_);
    libusb_context_ = NULL;
  }
}

int ODriveUSB::init(const std::vector<std::vector<int64_t>> & serial_numbers)
{
  RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "Initialisation ODriveUSB avec plusieurs cartes");
  
  int ret = libusb_init(&libusb_context_);
  if (ret != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Échec de l'initialisation de libusb: %s", libusb_error_name(ret));
    return ret;
  }

  // Afficher tous les numéros de série attendus depuis la configuration
  RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "Numéros de série attendus depuis la configuration:");
  for (size_t i = 0; i < serial_numbers.size(); ++i) {
    for (size_t j = 0; j < serial_numbers[i].size(); ++j) {
      RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "  Groupe %zu, Index %zu: %ld (0x%lX)", 
                 i, j, serial_numbers[i][j], serial_numbers[i][j]);
    }
  }

  libusb_device ** device_list;
  ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
  if (device_count <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Aucun périphérique USB détecté: %s", libusb_error_name(device_count));
    return device_count;
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "%ld périphériques USB détectés, recherche des ODrives...", device_count);

  int odrive_count = 0;
  int connected_count = 0;
  for (ssize_t i = 0; i < device_count; ++i) {
    libusb_device * device = device_list[i];
    libusb_device_descriptor descriptor;
    
    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS) {
      continue;
    }

    // Vérifier si c'est un ODrive (VendorID/ProductID)
    if (descriptor.idVendor == ODRIVE_USB_VENDORID && descriptor.idProduct == ODRIVE_USB_PRODUCTID) {
      odrive_count++;
      RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "ODrive #%d détecté", odrive_count);

      libusb_device_handle * device_handle;
      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("ODriveUSB"), "Échec de l'ouverture de l'ODrive #%d", odrive_count);
        continue;
      }

      // Détacher le pilote kernel si nécessaire
      if (libusb_kernel_driver_active(device_handle, 2) == 1) {
        if (libusb_detach_kernel_driver(device_handle, 2) != LIBUSB_SUCCESS) {
          RCLCPP_WARN(rclcpp::get_logger("ODriveUSB"), "Échec du détachement du pilote kernel pour ODrive #%d", odrive_count);
          libusb_close(device_handle);
          continue;
        }
      }

      // Revendiquer l'interface
      if (libusb_claim_interface(device_handle, 2) != LIBUSB_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("ODriveUSB"), "Échec de la revendication de l'interface pour ODrive #%d", odrive_count);
        libusb_close(device_handle);
        continue;
      }

      // Lire le numéro de série
      uint64_t serial_number;
      if (read(device_handle, SERIAL_NUMBER, serial_number) != LIBUSB_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("ODriveUSB"), "Échec de la lecture du numéro de série pour ODrive #%d", odrive_count);
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
        continue;
      }

      RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "ODrive #%d - Numéro de série lu: %lu (0x%lX)", 
                 odrive_count, serial_number, serial_number);

      // Vérifier si ce numéro de série est dans la liste demandée
      bool match_found = true;
      for (const auto & group : serial_numbers) {
        for (auto expected_serial : group) {
          // Convertir en uint64_t pour comparaison
          uint64_t expected_serial_uint = static_cast<uint64_t>(expected_serial);
          if (serial_number == expected_serial_uint) {
            match_found = true;
            RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "Correspondance trouvée: %lu (0x%lX)", serial_number, serial_number);
            break;
          }
        }
        if (match_found) break;
      }

      if (match_found) {
        odrive_map_[serial_number] = device_handle;
        connected_count++;
        RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "ODrive connecté avec SN: %lu (0x%lX)", serial_number, serial_number);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ODriveUSB"), "ODrive avec SN %lu (0x%lX) non demandé, ignoré", serial_number, serial_number);
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
      }
    }
  }

  libusb_free_device_list(device_list, 1);

  RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "ODrives détectés: %d, ODrives connectés: %d", odrive_count, connected_count);

  if (odrive_map_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Aucun ODrive valide détecté ou connecté");
    return LIBUSB_ERROR_NO_DEVICE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveUSB"), "%zu ODrive(s) initialisé(s) avec succès", odrive_map_.size());
  return LIBUSB_SUCCESS;
}

template <typename T>
int ODriveUSB::read(int64_t & serial_number, short endpoint_id, T & value)
{
  // Convertir le numéro de série en uint64_t pour la recherche
  uint64_t serial_uint = static_cast<uint64_t>(serial_number);
  
  if (odrive_map_.find(serial_uint) == odrive_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Tentative de lecture sur ODrive non connecté: %ld", serial_number);
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return read(odrive_map_[serial_uint], endpoint_id, value);
}

template <typename T>
int ODriveUSB::read(libusb_device_handle * odrive_handle, short endpoint_id, T & value)
{
  bytes request_payload;
  bytes response_payload;

  int ret = endpointOperation(
    odrive_handle, endpoint_id, sizeof(value), request_payload, response_payload, 1);
  if (ret != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Échec de la lecture endpoint %d: %s", endpoint_id, libusb_error_name(ret));
    return ret;
  }

  std::memcpy(&value, &response_payload[0], sizeof(value));

  return LIBUSB_SUCCESS;
}

template <typename T>
int ODriveUSB::write(int64_t & serial_number, short endpoint_id, const T & value)
{
  // Convertir le numéro de série en uint64_t pour la recherche
  uint64_t serial_uint = static_cast<uint64_t>(serial_number);
  
  if (odrive_map_.find(serial_uint) == odrive_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Tentative d'écriture sur ODrive non connecté: %ld", serial_number);
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return write(odrive_map_[serial_uint], endpoint_id, value);
}

template <typename T>
int ODriveUSB::write(libusb_device_handle * odrive_handle, short endpoint_id, const T & value)
{
  bytes request_payload;
  bytes response_payload;

  for (size_t i = 0; i < sizeof(value); i++) {
    request_payload.emplace_back(((unsigned char *)&value)[i]);
  }

  int ret = endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
  if (ret != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Échec de l'écriture endpoint %d: %s", endpoint_id, libusb_error_name(ret));
  }
  
  return ret;
}

int ODriveUSB::call(int64_t & serial_number, short endpoint_id)
{
  // Convertir le numéro de série en uint64_t pour la recherche
  uint64_t serial_uint = static_cast<uint64_t>(serial_number);
  
  if (odrive_map_.find(serial_uint) == odrive_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Tentative d'appel sur ODrive non connecté: %ld", serial_number);
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return call(odrive_map_[serial_uint], endpoint_id);
}

int ODriveUSB::call(libusb_device_handle * odrive_handle, short endpoint_id)
{
  bytes request_payload;
  bytes response_payload;

  int ret = endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
  if (ret != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveUSB"), "Échec de l'appel endpoint %d: %s", endpoint_id, libusb_error_name(ret));
  }
  
  return ret;
}

int ODriveUSB::endpointOperation(
  libusb_device_handle * odrive_handle, short endpoint_id, short response_size,
  bytes request_payload, bytes & response_payload, bool MSB)
{
  int transferred = 0;
  bytes response_packet;
  unsigned char response_data[ODRIVE_MAX_PACKET_SIZE] = {0};

  if (MSB) {
    endpoint_id |= 0x8000;
  }
  sequence_number_ = (sequence_number_ + 1) & 0x7fff;
  sequence_number_ |= LIBUSB_ENDPOINT_IN;
  short sequence_number = sequence_number_;

  bytes request_packet = encodePacket(sequence_number, endpoint_id, response_size, request_payload);

  int ret = libusb_bulk_transfer(
    odrive_handle, ODRIVE_OUT_ENDPOINT, request_packet.data(), request_packet.size(), &transferred,
    0);
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }

  if (MSB) {
    ret = libusb_bulk_transfer(
      odrive_handle, ODRIVE_IN_ENDPOINT, response_data, ODRIVE_MAX_PACKET_SIZE, &transferred, 0);
    if (ret != LIBUSB_SUCCESS) {
      return ret;
    }

    for (int i = 0; i < transferred; i++) {
      response_packet.emplace_back(response_data[i]);
    }

    response_payload = decodePacket(response_packet);
  }

  return LIBUSB_SUCCESS;
}

bytes ODriveUSB::encodePacket(
  short sequence_number, short endpoint_id, short response_size, const bytes & request_payload)
{
  bytes packet;

  packet.emplace_back((sequence_number >> 0) & 0xFF);
  packet.emplace_back((sequence_number >> 8) & 0xFF);
  packet.emplace_back((endpoint_id >> 0) & 0xFF);
  packet.emplace_back((endpoint_id >> 8) & 0xFF);
  packet.emplace_back((response_size >> 0) & 0xFF);
  packet.emplace_back((response_size >> 8) & 0xFF);

  for (uint8_t b : request_payload) {
    packet.emplace_back(b);
  }

  short crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : json_crc_;
  packet.emplace_back((crc >> 0) & 0xFF);
  packet.emplace_back((crc >> 8) & 0xFF);

  return packet;
}

bytes ODriveUSB::decodePacket(bytes & response_packet)
{
  bytes payload;

  for (bytes::size_type i = 2; i < response_packet.size(); ++i) {
    payload.emplace_back(response_packet[i]);
  }

  return payload;
}

template int ODriveUSB::read(int64_t &, short, bool &);
template int ODriveUSB::read(int64_t &, short, float &);
template int ODriveUSB::read(int64_t &, short, int32_t &);
template int ODriveUSB::read(int64_t &, short, int64_t &);
template int ODriveUSB::read(int64_t &, short, uint8_t &);
template int ODriveUSB::read(int64_t &, short, uint16_t &);
template int ODriveUSB::read(int64_t &, short, uint32_t &);
template int ODriveUSB::read(int64_t &, short, uint64_t &);

template int ODriveUSB::write(int64_t &, short, const bool &);
template int ODriveUSB::write(int64_t &, short, const float &);
template int ODriveUSB::write(int64_t &, short, const int32_t &);
template int ODriveUSB::write(int64_t &, short, const int64_t &);
template int ODriveUSB::write(int64_t &, short, const uint8_t &);
template int ODriveUSB::write(int64_t &, short, const uint16_t &);
template int ODriveUSB::write(int64_t &, short, const uint32_t &);
template int ODriveUSB::write(int64_t &, short, const uint64_t &);
}  // namespace odrive