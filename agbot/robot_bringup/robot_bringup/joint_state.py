from math import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher_costum')

        self.wheel_radius = 0.2
        self.wheel_separation = 0.8
        self.msg = Twist()

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(Odometry, 'odometry/filtered', self.listener_callback, qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.prev_time = self.get_clock().now()  # Initialize prev_time
        loop_rate = self.create_rate(10)

        # robot state
        self.plate_base_position = 0.0 
        self.left_wheel_position = 0.0 
        self.right_wheel_position = 0.0

        # message declarations    
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                
                msg = self.msg
                linear_velocity = msg.twist.twist.linear.x
                angular_velocity = msg.twist.twist.angular.z

                v_left = linear_velocity - (self.wheel_separation * angular_velocity) / 2
                v_right = linear_velocity + (self.wheel_separation * angular_velocity) / 2

                # Update wheel positions in radians based on velocity
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds * 1e-9
                self.left_wheel_position += v_left / self.wheel_radius * dt
                self.right_wheel_position += v_right / self.wheel_radius * dt
                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['plate_base_joint', 'left_wheel_joint', 'right_wheel_joint']
                joint_state.position = [self.plate_base_position, self.left_wheel_position, self.right_wheel_position]
                joint_state.velocity = [0.0, v_left / self.wheel_radius, v_right / self.wheel_radius]

                # update transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = msg.pose.pose.position.x
                odom_trans.transform.translation.y = msg.pose.pose.position.y
                odom_trans.transform.rotation = msg.pose.pose.orientation

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                self.prev_time = current_time  # Update prev_time

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def listener_callback(self, msg):
        self.msg = msg

def main():
    rclpy.init()
    node = StatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState

# class WheelVelocityCalculator(Node):

#     def __init__(self):
#         super().__init__('wheel_velocity_calculator')
        
#         # Declare and get parameters
#         self.declare_parameter("wheel_radius", 0.2)
#         self.declare_parameter("wheel_separation", 0.8)

#         self.wheel_radius = self.get_parameter("wheel_radius").value  # in meters
#         self.wheel_separation = self.get_parameter("wheel_separation").value  # in meters

#         self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.listener_callback, 10)

#         self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

#         self.prev_time = self.get_clock().now()
#         self.left_wheel_position = 0.0
#         self.right_wheel_position = 0.0
#         self.plate_base_position = 0.0

#     def listener_callback(self, msg):
#         # Extracting linear and angular velocity of the robot
#         linear_velocity = msg.twist.twist.linear.x
#         angular_velocity = msg.twist.twist.angular.z

#         # Calculating the linear velocity of the left and right wheels
#         v_left = linear_velocity - (self.wheel_separation * angular_velocity) / 2
#         v_right = linear_velocity + (self.wheel_separation * angular_velocity) / 2

#         # Update wheel positions in radians based on velocity
#         current_time = self.get_clock().now()
#         dt = (current_time - self.prev_time).nanoseconds * 1e-9
#         self.left_wheel_position += v_left / self.wheel_radius * dt
#         self.right_wheel_position += v_right / self.wheel_radius * dt

#         # Creating JointState message
#         joint_state_msg = JointState()
#         joint_state_msg.name = ['plate_base_joint', 'left_wheel_joint', 'right_wheel_joint']
#         joint_state_msg.position = [self.plate_base_position, self.left_wheel_position, self.right_wheel_position]
#         joint_state_msg.velocity = [0.0, v_left / self.wheel_radius, v_right / self.wheel_radius]  
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()

#         # Publishing the JointState message
#         self.joint_state_publisher.publish(joint_state_msg)

#         # Printing the velocities
#         self.get_logger().info(f'Left Wheel Velocity: {v_left} m/s')
#         self.get_logger().info(f'Right Wheel Velocity: {v_right} m/s')

#         # Update the previous time to current
#         self.prev_time = current_time


# def main(args=None):
#     rclpy.init(args=args)
#     wheel_velocity_calculator = WheelVelocityCalculator()
#     rclpy.spin(wheel_velocity_calculator)

#     wheel_velocity_calculator.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()