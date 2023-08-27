import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String 

class ZeroTwistPublisher(Node):

    def __init__(self):
        super().__init__('zero_twist_publisher')
        self.odom_pub = self.create_publisher(TwistWithCovarianceStamped, 'wheel/odometry', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        odom = TwistWithCovarianceStamped()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "base_link"
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.odom_pub.publish(odom)

class EncoderTwistPublisher(Node):

    def __init__(self):
        super().__init__('encoder_twist_publisher')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.encoder_data_sub = self.create_subscription(String, 'wheel/odometry', self.encoder_data_callback, 10)

    def encoder_data_callback(self, msg):
        # Convert encoder data to velocities
        vx, vy, vth = self.convert_encoder_data_to_velocity(msg)

        # Create twist message
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0.0
        twist.angular.z = vth

        # Publish twist message
        self.twist_pub.publish(twist)

    def convert_encoder_data_to_velocity(self, msg):
        # Convert encoder data to velocities
        # This will depend on your specific robot configuration
        # ...
        return 0.0, 0.0, 0.0  # Modify this to convert actual encoder data

def main():
    rclpy.init()

    node = ZeroTwistPublisher()
    # node = EncoderTwistPublisher()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
