import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from custom_interfaces.msg import WheelEncoders

class VelocityCalculator(Node):

    def __init__(self):
        super().__init__('robot_velocity_pub')
        self.subscription = self.create_subscription(WheelEncoders, 'wheel/encoders', self.encoder_callback, 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'wheel/odometry', 10)

    def encoder_callback(self, msg):
        gear_radius = 0.1
        wheel_separation = 0.8

        v_left = msg.left_encoder * gear_radius * -1 # the negative value is due to the orientation of the encoder
        v_right = msg.right_encoder * gear_radius

        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / wheel_separation

        # Create and populate the TwistWithCovarianceStamped message
        twist_stamped = TwistWithCovarianceStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"  # Assuming velocities are in the base_link frame

        twist_stamped.twist.twist.linear.x = v
        twist_stamped.twist.twist.angular.z = w

        # Assuming some arbitrary covariance values for the example.
        # You should adjust these based on your system's characteristics.
        twist_stamped.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01
        ]

        self.twist_pub.publish(twist_stamped)



def main(args=None):
    rclpy.init(args=args)
    velocity_calculator = VelocityCalculator()
    rclpy.spin(velocity_calculator)
    velocity_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()