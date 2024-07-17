openDog V3 ROS2 Humble:
The following project intends to integrate ROS2 into the openDog V3 of James Bruton (XRobots) and provide a simulation environment for testing of new functions.

Launch Gazebo simulation with stand:
ros2 launch opendog_launch opendog_stand_gazebo.launch.py 

Launch Gazebo simulation without stand:
ros2 launch opendog_launch opendog_nostand_gazebo.launch.py 


TODO list:

Add Inverse kinematics node into a control package

Add control node into control package

Add teleop for a logitech gamepad into a teleop package 
