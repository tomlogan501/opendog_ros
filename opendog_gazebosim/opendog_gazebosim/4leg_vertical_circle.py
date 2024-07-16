import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class QuadrupedWalkingController(Node):

    def __init__(self):
        super().__init__('quadruped_walking_controller')

        # Create publishers for each leg and avoid conflict with naming
        self.leg_publisher = [
            self.create_publisher(JointTrajectory, f'/joint_trajectory_controller/joint_trajectory', 10) # /joint_trajectory_controller/leg{i}_joint_trajectory
            for i in range(1, 5)
        ]

        self.timer = self.create_timer(0.1, self.timer_callback)  # Reduced timer period for smoother motion
        self.step_sequence = self.generate_vertical_circle_sequence(radius=5, num_points=36) 
        self.current_step = 0
        self.get_logger().info("Quadruped walking controller node has been started.")

    def generate_vertical_circle_sequence(self, radius, num_points):
        sequence = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            y = radius * math.cos(angle)
            z = radius * math.sin(angle)
            sequence.append([y, z]) 
        return sequence

    def timer_callback(self):
        if self.current_step < len(self.step_sequence):
            for i in range(4):  # Publish to each leg
                self.publish_trajectory(i, self.step_sequence[self.current_step])
            self.current_step += 1
        else:
            self.current_step = 0  # Reset to start the circle again

    def publish_trajectory(self, leg_index, positions):
        msg = JointTrajectory()
        # Modify the joint names for each leg accordingly
        if leg_index == 0:
            msg.joint_names = ['ul_upperleg_to_shoulder', 'ul_lowerleg_to_upperleg']  # Front Left Leg Joints
        elif leg_index == 1:
            msg.joint_names = ['ur_upperleg_to_shoulder', 'ur_lowerleg_to_upperleg']  # Front Right Leg Joints
        elif leg_index == 2:
            msg.joint_names = ['ll_upperleg_to_hip', 'll_lowerleg_to_upperleg']  # Back Left Leg Joints
        elif leg_index == 3:
            msg.joint_names = ['lr_upperleg_to_hip', 'lr_lowerleg_to_upperleg']  # Back Right Leg Joints

        point = JointTrajectoryPoint()
        # Assume the trajectory only sets positions for y and z axes, and leave the x-axis zero
        point.positions = [0.0, positions[0], positions[1]] 
        point.time_from_start.sec = 1  # Execute over 1 second

        msg.points.append(point)

        self.leg_publisher[leg_index].publish(msg)
        self.get_logger().info(f"Published joints trajectory command for leg {leg_index + 1}: {positions}")

def main(args=None):
    rclpy.init(args=args)
    quadruped_walking_controller = QuadrupedWalkingController()
    rclpy.spin(quadruped_walking_controller)
    quadruped_walking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

