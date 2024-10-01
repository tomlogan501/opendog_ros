# opendog_ros
Opendog_ROS is a quadruped robot which is fully based on ROS 2

The following project intends to integrate ROS2 into the openDog V3 of James Bruton (XRobots https://github.com/XRobots/openDogV3) with another robot project opendog of Nipun Dhananjaya (https://github.com/NDHANA94/opendog_ros2)  and provide a simulation environment for testing of new functions.

## Contains
This repository contains ros2 packages for quadruped robot opendog_ros.
packages are :
  1. **`opendog_msgs`** : this package contains the msgs those used by other packages.
  
        1. `JoyCtrlCmds` : this contains the control variables of the robot from the gamepad
              - `bool[3] states` : { start, walk, side_move_mode} 
              - `uint8 gait_type` : to change the gait type
              - `geometry_msgs/Pose pose` : to control slant(x,y) and roll,pitch,yaw
              - `geometry_msgs/Vector3 gait_step` : gait_step.x = steplen_x, gait_step.y = steplen_y, gait_step.z = swing_height
              
        2. `Geometry`: this contains the parameters for coordinate of each leg and body orientation(roll,pitch,yaw)
              - `geometry_msgs/Point32 fr` : x,y,z end effector coordinates of FR leg
              - `geometry_msgs/Point32 fl` : x,y,z end effector coordinates of FL leg
              - `geometry_msgs/Point32 br` : x,y,z end effector coordinates of BR leg
              - `geometry_msgs/Point32 bl` : x,y,z end effector coordinates of BL leg
              - `geometry_msgs/Quaternion euler_ang` : roll, pitch, yaw angles
              
  2. **`opendog_teleop`** : this pkg creates `/opendog_teleop_gamepad_node. 
        - Node 1 : `/joy_node`
            This node creates commands to robot from Gamepad commands
            - subscriber : `/joy_node` 
            - publisher : `/opendog_joy_ctrl_cmd` using the interface `opendog_msgs/msg/JoyCtrlCmd`

  3. **`opendog_ctrl`** : This pkg has `Body_motion_planner` and `gait_generater` and creates the nodes `/command_manager_node` and `/IK_node`
        - `Body_motion_planner` : plans body motions from control commands receive from `/command_manager_node`
        - `gait_generator` : generates gaits acording to the given gait_type command from the Gamepad 
   
        - Node 1 : `/command_manager_node` 
            - subscriber : `/opendog_joy_ctrl_cmd` via `opendog_msgs/msg/JoyCtrlCmds` interface
            - publisher : `/opendog_geometry` via `opendog_msgs/msg/Geometry` interface

        - Node 2 : `/IK_node`
            - subscriber : `/opendog_geometry` via `opendog_msgs/msg/Geometry` interface
            - publisher  : `/opendog_jointController/commands`
  

  4. **`opendog_launch`** : This contains the launch file for all the above nodes and `micro_ros_agent`
  
  5. **`opendog_gazebo_sim`** : Gazebo simmulation 
  
  6. **`opendog_gazebo_joint_cmd`** : this pkg contains the node `/opendog_gazebo_joint_cmd` to send joint angles to gazebo
        - Node : `/opendog_gazebo_joint_cmd`
            - subscriber : `/opendog_jointController/commands`
            - publisher : '/gazebo_joint_controller/commands`

 # Launching
 source the workspace  
 ```
 source opendog_ws/install/setup.bash
 ```
 to add workspace source permenently to .bashrc:
  ```
    
    # add source 
    echo "source /home/$USER/opendog_ws/install/setup.bash" >> ~/.bachrc
  ```
  
  to launch run following 
  ```
  ros2 launch opendog_launch opendog.launch.py
 
  ```
  
  to launch gazebo with opendog
  ```
  ros2 launch opendog_gazebo_sim opendog_gazebo_sim.launch.py
  ```

	Gamepad control:
	Start:	wake up, sleep
	Y,B	gait type (Y (walk), B(trot)) 
	Select:	gait Start/Stop
	
	Right joystick: increase movement pace, walking direction
	Left joystick:	Pitch (Y axis) and Roll (X axis)
	D-pad:		Change height of robot (Y axis)
	LT, RT:		Yaw Counter Clock Wise, Yaw Clock Wise

 ![image](https://github.com/user-attachments/assets/5d14f580-36b9-421a-a2bb-c1c6768295e5)

	
