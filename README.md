openDog V3 ROS2 Humble:
The following project intends to integrate ROS2 into the openDog V3 of James Bruton (XRobots) and provide a simulation environment for testing of new functions.

**Launch Gazebo simulation with stand:**
ros2 launch opendog_launch opendog_stand_gazebo.launch.py 

**Launch Gazebo simulation without stand:**
ros2 launch opendog_launch opendog_nostand_gazebo.launch.py 

**Launch Gazebo control simulation:**
ros2 launch opendog_gazebosim opendog_gazebosim.launch.py 
ros2 launch opendog_launch opendog.launch.py

	Gamepad control:
	Start:	wake up, sleep
	Y,B	gait type (Y (walk), B(trot)) 
	Select:	gait Start/Stop
	
	Right joystick: increase movement pace, walking direction
	Left joystick:	Pitch (Y axis) and Roll (X axis)
	D-pad:		Change height of robot (Y axis)
	LT, RT:		Yaw Counter Clock Wise, Yaw Clock Wise
	

