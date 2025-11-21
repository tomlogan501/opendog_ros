cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control_can)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(odrive_ros2_control_can SHARED
  src/odrive_can.cpp
  src/odrive_hardware_interface_can.cpp
)

ament_target_dependencies(odrive_ros2_control_can
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

target_include_directories(odrive_ros2_control_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS odrive_ros2_control_can
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES odrive_hardware_can_plugin.xml
  DESTINATION share/odrive_ros2_control_can
)

# Export du plugin pour pluginlib
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_can_plugin.xml)

ament_package()
