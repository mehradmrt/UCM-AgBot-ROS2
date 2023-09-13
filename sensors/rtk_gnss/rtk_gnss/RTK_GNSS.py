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

        self.declare_parameter("serial_port", "/dev/reachRS2")  # Default value
        self.declare_parameter("baudrate", 115200)  # Default value

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info('Successfully connected to {}'.format(self.serial_port))
        except serial.SerialException as err:
            self.get_logger().error('Failed to connect to {}. Error: {}'.format(self.serial_port, str(err)))
            return

        time_period = 0.2
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
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.ip_address = "192.168.0.213"  # RS IP:192.168.0.222(RS+) and [.223(RS2) for robot_AP] and [.213 for ehsani_lab] 
        self.port = 9001
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sensor_timeout = 2  # seconds
        self.last_received_time = None

        self.establish_connection()

        self.data_timer_period = 0.2
        self.data_timer = self.create_timer(self.data_timer_period, self.read_and_publish_gps_info)

        self.reconnect_timer_period = 1  # seconds
        self.reconnect_timer = self.create_timer(self.reconnect_timer_period, self.check_connection)

    def establish_connection(self):
        try:
            self.sock.connect((self.ip_address, self.port))
            self.get_logger().info('Successfully connected to {}:{}'.format(self.ip_address, self.port))
            self.last_received_time = self.get_clock().now().nanoseconds
        except socket.error as err:
            self.get_logger().warn('No route to host {}:{}. Retrying ...'.format(self.ip_address, self.port))
    
    def reset_connection(self):
        self.sock.close()
        self.sock = None
        self.sock = self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().warn('Connection is reset... {}:{}'.format(self.ip_address, self.port))

    def check_connection(self):
        current_time = self.get_clock().now().nanoseconds
        if self.last_received_time and (current_time - self.last_received_time) * 1e-9 > self.sensor_timeout:
            print((current_time-self.last_received_time)* 1e-9)
            self.get_logger().warn('Sensor timeout detected. Attempting to reconnect...')
            self.reset_connection()
            self.establish_connection()
        elif not self.last_received_time:
            self.reset_connection()
            self.establish_connection()

    def read_and_publish_gps_info(self):
        try:
            data = self.sock.recv(1024)
            if data:
                self.last_received_time = self.get_clock().now().nanoseconds
                lines = data.decode("utf-8").rstrip().split('\r\n')
                for line in lines:
                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            gps_msg = NavSatFix()
                            gps_msg.header.stamp = self.get_clock().now().to_msg()
                            gps_msg.header.frame_id = "gps_link"
                            gps_msg.latitude = msg.latitude 
                            gps_msg.longitude = msg.longitude
                            gps_msg.altitude = float(msg.altitude) if hasattr(msg, 'altitude') and msg.altitude else 0.0
                            self.publisher_.publish(gps_msg)
                            # self.get_logger().info('Publishing GPS Info: "%s"' % str(gps_msg))
                        except pynmea2.ChecksumError:
                            self.get_logger().warn('Invalid NMEA sentence checksum. Ignoring sentence.')
                            self.check_connection()
                        except pynmea2.ParseError as e:
                            self.get_logger().error('Failed to parse NMEA sentence. Error: {}'.format(str(e)))
                            self.check_connection()
                        except ValueError as ve:
                            self.get_logger().warn(f"ValueError encountered: {ve}. Ignoring sentence.")
                            self.check_connection()
        except socket.error as se:
            self.get_logger().error('Socket error: {}'.format(str(se)))
            self.check_connection()
        



def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GPSPublisher_TCP() 
    # gps_publisher = GPSPublisher_ser()

    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
