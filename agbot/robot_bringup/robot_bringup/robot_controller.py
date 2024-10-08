import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading
import socket


class RobotCommands:
    def __init__(self):
        self.server_ip = "192.168.1.7"
        self.port = 20001
        self.socket = None

        # Initialize the 13-byte data packet with default values
        self.data = bytearray([0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38])

    def is_connected(self):
        return self.socket is not None

    def connection(self):
        while not self.is_connected():
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.port))
                print("Connection established")
            except socket.error as e:
                print(f"Failed to connect: {e}. Retrying...")
                time.sleep(1)

    def set_left_speed(self, speed):
        self.data[9] = speed

    def set_right_speed(self, speed):
        self.data[10] = speed

    def set_direction_ctrl(self, state):
        self.data[11] = state

    def sending_data(self):
        if not self.is_connected():
            print("No connection. Attempting to reconnect...")
            self.connection()

        try:
            self.socket.sendall(self.data)
            print(f"Sending data: {list(self.data)}")
        except socket.error as e:
            print(f"Failed to send data: {e}")
            self.socket.close()
            self.socket = None  # Mark as disconnected


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.robot = RobotCommands()

        # Subscribe to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        self.robot.connection()  # Establish connection to the robot

        # Mutex to ensure thread safety during data transmission
        self.robot_mutex = threading.Lock()

        # Start a background thread to handle repeated data transmission every 20ms
        self.running = True
        self.transmit_thread = threading.Thread(target=self.repeat_transmit)
        self.transmit_thread.start()

    def listener_callback(self, msg):
        linear_vel = msg.linear.x  # Forward/backward motion
        angular_vel = msg.angular.z  # Rotation motion

        print(f"Linear: {linear_vel}, Angular: {angular_vel}")

        # Calculate motor speeds
        left_speed, right_speed = self.calculate_motor_speeds(linear_vel, angular_vel)

        # Set the motor speeds and direction
        self.set_speed_and_direction(left_speed, right_speed)

    def calculate_motor_speeds(self, linear_vel, angular_vel):
        wheel_distance = 0.8  # Distance between the wheels

        # Adjust left and right motor speeds based on linear and angular velocity
        left_speed = linear_vel - (angular_vel * wheel_distance / 2.0)
        right_speed = linear_vel + (angular_vel * wheel_distance / 2.0)

        return left_speed, right_speed

    def set_speed_and_direction(self, left_speed, right_speed):
        # Map the speeds to motor control values
        left_motor_value = self.map_to_motor_range(left_speed)
        right_motor_value = self.map_to_motor_range(right_speed)

        # Determine the direction of rotation (0 for forward, 1 for clockwise, etc.)
        direction = self.determine_direction(left_speed, right_speed)

        # Apply the direction and motor speeds
        with self.robot_mutex:
            self.robot.set_direction_ctrl(direction)
            self.robot.set_left_speed(left_motor_value)
            self.robot.set_right_speed(right_motor_value)

    def map_to_motor_range(self, speed):
        max_speed_value = 250
        min_speed_value = 50

        speed = abs(speed)
        if speed == 0:
            return 0  # Stop motor if speed is 0
        return int(max(min_speed_value, min(max_speed_value, speed * (max_speed_value - min_speed_value) + min_speed_value)))

    def determine_direction(self, left_speed, right_speed):
        if left_speed > 0 and right_speed > 0:
            return 0  # Forward
        elif left_speed < 0 and right_speed < 0:
            return 3  # Reverse
        elif left_speed > 0 and right_speed < 0:
            return 1  # Clockwise (rotation)
        elif left_speed < 0 and right_speed > 0:
            return 2  # Counterclockwise (rotation)
        return 0  # Default to forward

    def repeat_transmit(self):
        # Continuously send motor commands every 20ms
        while self.running:
            with self.robot_mutex:
                self.robot.sending_data()
            time.sleep(0.02)  # Sleep for 20 milliseconds

    def stop(self):
        # Stop the background thread when the node is shutting down
        self.running = False
        self.transmit_thread.join()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)  # Spin the node
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.stop()  # Stop the background thread
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
