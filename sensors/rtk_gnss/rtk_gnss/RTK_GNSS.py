import rclpy
import socket
import serial
import pynmea2
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSPublisher_ser(Node):
    def __init__(self):
        super().__init__('gps_subpub')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

        self.declare_parameter("serial_port", "/dev/ttyACM0")  # Default value
        self.declare_parameter("baudrate", 115200)  # Default value

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info('Successfully connected to {}'.format(self.serial_port))
        except serial.SerialException as err:
            self.get_logger().error('Failed to connect to {}. Error: {}'.format(self.serial_port, str(err)))
            return

        time_period = 0.1
        self.timer = self.create_timer(time_period, self.read_and_publish_gps_info)

    def read_and_publish_gps_info(self):
        # self.get_logger().info('In read_and_publish_gps_info...')
        try:
            data = self.serial_conn.readline()
            if data:
                # print(data)
                line = data.decode("utf-8").rstrip()
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    # print(line)
                    msg = pynmea2.parse(line)
                    self.get_logger().info('Publishing GPS Info: "%s"' % str(msg))
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.header.frame_id = "gps_link"
                    gps_msg.latitude = msg.latitude
                    gps_msg.longitude = msg.longitude
                    gps_msg.altitude = float(msg.altitude) if hasattr(msg, 'altitude') and msg.altitude else 0.0
                    self.publisher_.publish(gps_msg)
                    self.get_logger().info('Publishing GPS Info: "%s"' % str(gps_msg))
        except pynmea2.ParseError as e:
            self.get_logger().error('Failed to parse NMEA sentence. Error: {}'.format(str(e)))


class GPSPublisher_TCP(Node):
    def __init__(self):
        super().__init__('gps_subpub')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 10)
        self.ip_address = "192.168.42.1"  # RS IP:192.168.0.222 , port: 9001 ---or---- IP:192.168.42.1 
        self.port = 9001
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.ip_address, self.port))
            self.get_logger().info('Successfully connected to {}:{}'.format(self.ip_address, self.port))
        except socket.error as err:
            self.get_logger().error('Failed to connect to {}:{}. Error: {}'.format(self.ip_address, self.port, str(err)))
            return
        time_period = 1
        self.timer = self.create_timer(time_period, self.read_and_publish_gps_info)
        
    def read_and_publish_gps_info(self):
        try:
            data = self.sock.recv(1024)
            if data:
                # print(data)
                line = data.decode("utf-8").rstrip()
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    # print(line)
                    msg = pynmea2.parse(line)
                    self.get_logger().info('Publishing GPS Info: "%s"' % str(msg))
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.header.frame_id = "gps"
                    gps_msg.latitude = msg.latitude
                    gps_msg.longitude = msg.longitude
                    gps_msg.altitude = float(msg.altitude) if hasattr(msg, 'altitude') and msg.altitude else 0.0
                    self.publisher_.publish(gps_msg)
                    self.get_logger().info('Publishing GPS Info: "%s"' % str(gps_msg))
        except pynmea2.ParseError as e:
            self.get_logger().error('Failed to parse NMEA sentence. Error: {}'.format(str(e)))


def main(args=None):
    rclpy.init(args=args)

    # gps_publisher = GPSPublisher_TCP() # TCP/IP is very unstable with EMLID 
    gps_publisher = GPSPublisher_ser()

    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
