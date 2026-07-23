cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control_can)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(signal_test_node src/signal_test_node.cpp)
ament_target_dependencies(signal_test_node rclcpp std_msgs sensor_msgs)

install(TARGETS odrive_ros2_control_can
  LIBRARY DESTINATION lib
)

install(TARGETS signal_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES odrive_hardware_can_plugin.xml
  DESTINATION share/odrive_ros2_control_can
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_can_plugin.xml)

install(DIRECTORY test/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY test/config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY test/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()

