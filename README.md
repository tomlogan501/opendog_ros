The following packages were used for testing purposes of a single leg of the quadruped robot openDog V3 in ROS 2 Humble.

Launch Gazebo simulation:
ros2 launch openleg_description gazebo.launch.py

ros2 run openleg_description leg_vertical_circle

To run on hardware:

1 leg ) 

        ros2 launch odrive_demo_bringup odrive_rrbot.launch.py

        ros2 run openleg_description leg_vertical_circle

2 legs) 

        ros2 launch odrive_demo_bringup odrive_rrbot2.launch.py

        ros2 run openleg_description 2leg_vertical_circle

4 legs) 

        ros2 launch odrive_demo_bringup odrive_rrbot4.launch.py

        ros2 run openleg_description 4leg_vertical_circle




