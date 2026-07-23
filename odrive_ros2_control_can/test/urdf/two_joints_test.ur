<?xml version="1.0"?>
<robot name="two_joints_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link"/>
  <link name="joint0_link"/>
  <joint name="motor_joint_0" type="revolute">
    <parent link="base_link"/>
    <child link="joint0_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
  <link name="joint1_link"/>
  <joint name="motor_joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="joint1_link"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
  <ros2_control name="two_joints_test" type="system">
    <hardware>
      <plugin>odrive_ros2_control_can/ODriveHardwareInterfaceCAN</plugin>
      <param name="can_interface">can0</param>
    </hardware>
    <joint name="motor_joint_0">
      <param name="node_id">0</param>
      <param name="axis">0</param>
      <param name="torque_constant">8.27</param>
      <param name="enable_watchdog">false</param>
      <param name="watchdog_timeout">0.1</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="motor_joint_1">
      <param name="node_id">1</param>
      <param name="axis">1</param>
      <param name="torque_constant">8.27</param>
      <param name="gear_ratio">10.0</param>
      <param name="enable_watchdog">false</param>
      <param name="watchdog_timeout">0.1</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
