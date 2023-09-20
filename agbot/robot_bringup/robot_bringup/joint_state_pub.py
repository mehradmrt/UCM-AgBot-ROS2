import rclpy
import tf2_py
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from custom_interfaces.msg import WheelEncoders

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_pub')
        self.subscription = self.create_subscription(WheelEncoders, 'wheel/encoders', self.encoder_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.prev_time = self.get_clock().now()

        self.br = TransformBroadcaster(self)


    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        delta_t = (current_time - self.prev_time).nanoseconds * 1e-9
        # print(delta_t)

        delta_left = -msg.left_encoder * delta_t
        delta_right = msg.right_encoder * delta_t

        self.left_wheel_position += delta_left
        self.right_wheel_position += delta_right

        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]

        self.joint_pub.publish(joint_state)
        self.prev_time = current_time

        # self.broadcast_joint_transforms(current_time)

    def broadcast_joint_transforms(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "base_link"

        t.child_frame_id = "left_wheel_link"
        q = self.euler_to_quaternion(0, self.left_wheel_position, 0)
        t.transform.rotation = q
        self.br.sendTransform(t)

        t.child_frame_id = "right_wheel_link"
        q = self.euler_to_quaternion(0, self.right_wheel_position, 0)
        t.transform.rotation = q
        self.br.sendTransform(t)

    def euler_to_quaternion(roll, pitch, yaw):
        quat = tf2_py.Quaternion()
        quat.setRPY(roll, pitch, yaw)
        return Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)


def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
