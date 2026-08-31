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

#ifndef ODRIVE_HARDWARE_INTERFACE_CAN__VISIBILITY_CONTROL_HPP_
#define ODRIVE_HARDWARE_INTERFACE_CAN__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ODRIVE_HARDWARE_INTERFACE_CAN_EXPORT __attribute__((dllexport))
#define ODRIVE_HARDWARE_INTERFACE_CAN_IMPORT __attribute__((dllimport))
#else
#define ODRIVE_HARDWARE_INTERFACE_CAN_EXPORT __declspec(dllexport)
#define ODRIVE_HARDWARE_INTERFACE_CAN_IMPORT __declspec(dllimport)
#endif
#ifdef ODRIVE_HARDWARE_INTERFACE_CAN_BUILDING_LIBRARY
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC ODRIVE_HARDWARE_INTERFACE_CAN_EXPORT
#else
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC ODRIVE_HARDWARE_INTERFACE_CAN_IMPORT
#endif
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC_TYPE ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC
#define ODRIVE_HARDWARE_INTERFACE_CAN_LOCAL
#else
#define ODRIVE_HARDWARE_INTERFACE_CAN_EXPORT __attribute__((visibility("default")))
#define ODRIVE_HARDWARE_INTERFACE_CAN_IMPORT
#if __GNUC__ >= 4
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC __attribute__((visibility("default")))
#define ODRIVE_HARDWARE_INTERFACE_CAN_LOCAL __attribute__((visibility("hidden")))
#else
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC
#define ODRIVE_HARDWARE_INTERFACE_CAN_LOCAL
#endif
#define ODRIVE_HARDWARE_INTERFACE_CAN_PUBLIC_TYPE
#endif

#endif  // ODRIVE_HARDWARE_INTERFACE_CAN__VISIBILITY_CONTROL_HPP_