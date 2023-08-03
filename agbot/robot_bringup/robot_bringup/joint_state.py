import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class WheelVelocityCalculator(Node):

    def __init__(self):
        super().__init__('wheel_velocity_calculator')
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.listener_callback, 10)

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.wheel_radius = 0.2  # in meters
        self.wheel_separation = 0.8  # in meters

        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.plate_base_position = 0.0

    def listener_callback(self, msg):
        # Extracting linear and angular velocity of the robot
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # Calculating the linear velocity of the left and right wheels
        v_left = linear_velocity - (self.wheel_separation * angular_velocity) / 2
        v_right = linear_velocity + (self.wheel_separation * angular_velocity) / 2

        # Update wheel positions in radians based on velocity
        dt = 1.0 / 5  # Assuming 10 Hz rate
        self.left_wheel_position += v_left / self.wheel_radius * dt
        self.right_wheel_position += v_right / self.wheel_radius * dt

        # Creating JointState message
        joint_state_msg = JointState()
        joint_state_msg.name = ['plate_base_joint', 'left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [self.plate_base_position, self.left_wheel_position, self.right_wheel_position]
        joint_state_msg.velocity = [0.0, v_left / self.wheel_radius, v_right / self.wheel_radius]  
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publishing the JointState message
        self.joint_state_publisher.publish(joint_state_msg)

        # Printing the velocities
        self.get_logger().info(f'Left Wheel Velocity: {v_left} m/s')
        self.get_logger().info(f'Right Wheel Velocity: {v_right} m/s')


def main(args=None):
    rclpy.init(args=args)
    wheel_velocity_calculator = WheelVelocityCalculator()
    rclpy.spin(wheel_velocity_calculator)

    wheel_velocity_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
