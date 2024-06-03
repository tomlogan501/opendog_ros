import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class LegWalkingController(Node):

    def __init__(self):
        super().__init__('leg_walking_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.step_sequence = [
            [0.0, 0.0, 0.0],  # Initial Position
            [0.2, -0.2, 0.2], # Lift Leg
            [0.0, -0.4, 0.4], # Move Forward
            [0.2, -0.2, 0.2], # Lower Leg
            [0.0, 0.0, 0.0]   # Back to initial position
        ]
        self.current_step = 0
        self.get_logger().info("Leg walking controller node has been started.")

    def timer_callback(self):
        if self.current_step < len(self.step_sequence):
            self.publish_trajectory(self.step_sequence[self.current_step])
            self.current_step += 1
        else:
            self.current_step = 0 # Reset to start the walking cycle again

    def publish_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['ul_shoulder_to_base_link', 'ul_upperleg_to_shoulder', 'ul_lowerleg_to_upperleg']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1 # Execute over 1 second

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