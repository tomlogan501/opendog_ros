import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class LegWalkingController(Node):

    def __init__(self):
        super().__init__('leg_walking_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Reduced timer period for smoother motion
        self.step_sequence = self.generate_circle_sequence(radius=0.2, num_points=36)
        self.current_step = 0
        self.get_logger().info("Leg walking controller node has been started.")

    def generate_circle_sequence(self, radius, num_points):
        sequence = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 0.0  # Assuming movement in a plane parallel to the ground
            sequence.append([x, y, z])
        return sequence

    def timer_callback(self):
        if self.current_step < len(self.step_sequence):
            self.publish_trajectory(self.step_sequence[self.current_step])
            self.current_step += 1
        else:
            self.current_step = 0  # Reset to start the circle again

    def publish_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['ul_shoulder_to_base_link', 'ul_upperleg_to_shoulder', 'ul_lowerleg_to_upperleg']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1  # Execute over 1 second

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint trajectory command: {positions}")

def main(args=None):
    rclpy.init(args=args)
    leg_walking_controller = LegWalkingController()
    rclpy.spin(leg_walking_controller)
    leg_walking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

