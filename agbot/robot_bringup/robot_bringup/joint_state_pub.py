import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_interfaces.msg import WheelEncoders

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_pub')
        self.subscription = self.create_subscription(WheelEncoders, 'wheel/encoders', self.encoder_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Initial positions and previous timestamp
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.prev_time = self.get_clock().now()

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        delta_t = (current_time - self.prev_time).nanoseconds * 1e-9
        # print(delta_t)

        # Calculate the change in position based on the encoder readings
        delta_left = -msg.left_encoder * delta_t
        delta_right = msg.right_encoder * delta_t

        # Update the joint positions
        self.left_wheel_position += delta_left
        self.right_wheel_position += delta_right

        # Create and populate the JointState message
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]

        self.joint_pub.publish(joint_state)

        # Update the previous time
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
