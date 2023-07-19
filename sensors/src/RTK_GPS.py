import rclpy
import socket
import pynmea2
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 10)
        self.ip_address = "192.168.1.100"  
        self.port = 0000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip_address, self.port))
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.read_and_publish_gps_info)

    def read_and_publish_gps_info(self):
        data = self.sock.recv(1024)
        lines = data.decode("utf-8").split('\r\n')
        for line in lines:
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = "gps"
                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = msg.altitude
                self.publisher_.publish(gps_msg)
                self.get_logger().info('Publishing GPS Info: "%s"' % str(gps_msg))





def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GPSPublisher()

    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()