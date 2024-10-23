#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import pyproj

class PathPlotter(Node):

    def __init__(self):
        super().__init__('path_plotter')

        # Initialize variables for odometry
        self.odom_x_positions = []
        self.odom_y_positions = []

        # Initialize variables for GPS
        self.lat0, self.lon0 = None, None
        self.gps_x_positions = []
        self.gps_y_positions = []

        # Subscriptions for odometry and GPS
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odometry_callback,
            10
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

    def latlon_to_xy(self, lat, lon, lat0, lon0):
        geod = pyproj.Geod(ellps='WGS84')
        fwd_azimuth, back_azimuth, distance = geod.inv(lon0, lat0, lon, lat)
        angle_rad = np.deg2rad(fwd_azimuth)
        x = distance * np.cos(angle_rad)
        y = distance * np.sin(angle_rad)
        return x, y

    def odometry_callback(self, msg):
        # Extract the position from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.get_logger().info(f'Received odom data: x={x}, y={y}')

        self.odom_x_positions.append(x)
        self.odom_y_positions.append(y)

    def gps_callback(self, data):
        if self.lat0 is None and self.lon0 is None:
            # Save the initial GPS point
            self.lat0, self.lon0 = data.latitude, data.longitude
            self.get_logger().info(f'Initial GPS position set: lat={self.lat0}, lon={self.lon0}')

        # Convert GPS lat/lon to x/y coordinates relative to the initial point
        x, y = self.latlon_to_xy(data.latitude, data.longitude, self.lat0, self.lon0)
        self.gps_x_positions.append(-x)
        self.gps_y_positions.append(-y)
        self.get_logger().info(f'GPS point added: x={x}, y={y}')

    def plot_paths(self):
        # Plot both the odometry and GPS paths on the same figure
        plt.figure()
        # Plot Odometry data
        plt.plot(self.odom_x_positions, self.odom_y_positions, label="Odometry Path", color='b')
        # Plot GPS data
        plt.plot(self.gps_y_positions, self.gps_x_positions, label="GPS Path", color='r', marker='o', linestyle='--')
        plt.xlabel('X Position (meters)')
        plt.ylabel('Y Position (meters)')
        plt.title('Comparison of Robot Path from Odometry and GPS Data')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    path_plotter = PathPlotter()

    try:
        # Spin the node until shutdown
        while rclpy.ok():
            rclpy.spin_once(path_plotter)
    except KeyboardInterrupt:
        pass

    # After playback, plot both paths
    path_plotter.plot_paths()

    path_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
