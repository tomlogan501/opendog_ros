cmake_minimum_required(VERSION 3.8)
project(odrive_can_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(odrive_can_system SHARED
  src/odrive_can.cpp
  src/odrive_hardware_interface_can.cpp
)

ament_target_dependencies(odrive_can_system
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

install(TARGETS odrive_can_system
  LIBRARY DESTINATION lib
)

# Installer le plugin xml pour pluginlib
install(FILES odrive_hardware_can_plugin.xml
  DESTINATION share/odrive_can_system
)

ament_package()

# --- ajout pour includes publics et installation des headers ---
target_include_directories(odrive_can_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# installer tout le contenu de include/ en respectant la hiérarchie
install(DIRECTORY include/
  DESTINATION include
)
# --- fin ajout ---
