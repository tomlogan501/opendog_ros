import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class LegWalkingController(Node):

    def __init__(self):
        super().__init__('leg_walking_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)
        self.timer = self.create_timer(0.15, self.timer_callback)  # Shorter timer period for smoother motion 0.05
        self.step_sequence = self.generate_vertical_circle_sequence(radius=5, num_points=20)  # Increased num_points for smoother circle
        self.current_step = 0
        self.get_logger().info("Leg walking controller node has been started.")

    def generate_vertical_circle_sequence(self, radius, num_points):
        sequence = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = 0.0  # Assuming movement in a vertical plane, keep x constant #Hip
            y = radius * math.cos(angle)  #Lower leg	
            z = radius * math.sin(angle)  # Upper Leg
            sequence.append([y, z, y, z, y, z, y, z])  # [x, y, z]
        return sequence

    def timer_callback(self):
        if self.current_step < len(self.step_sequence):
            self.publish_trajectory(self.step_sequence[self.current_step])
            self.current_step += 1
        else:
            self.current_step = 0  # Reset to start the circle again

    def publish_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint5', 'joint6', 'joint7', 'joint8', 'joint11', 'joint12']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0  # Use 0 seconds and define time in nanoseconds for finer control
        point.time_from_start.nanosec = int(0.15 * 1e9)  # 0.05 seconds execution time (matching timer period)

        # Optional: Add velocity and acceleration to smooth out transitions
        #point.velocities = [0.7, 0.7, 0.7, 0.7]  # Adjust as necessary
        #point.accelerations = [0.2, 0.2, 0.2, 0.2]  # Adjust as necessary

        msg.points.append(point)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    leg_walking_controller = LegWalkingController()
    rclpy.spin(leg_walking_controller)
    leg_walking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

