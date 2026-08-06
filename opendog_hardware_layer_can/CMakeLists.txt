cmake_minimum_required(VERSION 3.8)
project(opendog_hardware_layer_can)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(opendog_hardware_joint_ctrl_node src/opendog_hardware_joint_ctrl.cpp)
ament_target_dependencies(opendog_hardware_joint_ctrl_node rclcpp std_msgs)

add_executable(br_leg_test_joint_ctrl_node src/br_leg_test_joint_ctrl.cpp)
ament_target_dependencies(br_leg_test_joint_ctrl_node rclcpp std_msgs)

add_executable(bl_leg_test_joint_ctrl_node src/bl_leg_test_joint_ctrl.cpp)
ament_target_dependencies(bl_leg_test_joint_ctrl_node rclcpp std_msgs)


install(TARGETS
  opendog_hardware_joint_ctrl_node
  br_leg_test_joint_ctrl_node
  bl_leg_test_joint_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
