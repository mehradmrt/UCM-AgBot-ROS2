import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from robot_bringup import motor_cmd

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        self.robot = motor_cmd.RobotControl()

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Parameters specific to your robot
        max_speed = 2  # maximum speed in m/s
        wheel_distance = 0.8  # distance between wheels in meters

        # Convert Twist commands to differential drive motor commands
        self.motor_left = (linear_vel - angular_vel * wheel_distance / 2.0) / max_speed
        self.motor_right = (linear_vel + angular_vel * wheel_distance / 2.0) / max_speed

        # Change motor command
        self.robot.set_motor_ctrl(self.motor_left, self.motor_right)
        
        # Sending motor command
        self.robot.sending_data()


def main(args=None):
    # init ros node
    rclpy.init(args=args)
    motor_sub = MotorSubscriber()

    print('Motor control')

    # robot connection
    motor_sub.robot.connection()

    # init command
    motor_sub.robot.set_left_speed(0)
    motor_sub.robot.set_right_speed(0)

    rclpy.spin(motor_sub)

if __name__ == '__main__':
    main()
