#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import yaml

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def send_waypoints(self, waypoints_file):
        # Load waypoints from YAML file
        with open(waypoints_file, 'r') as file:
            waypoints = yaml.safe_load(file)['waypoints']

        goal = FollowWaypoints.Goal()

        # Convert waypoints to PoseStamped format
        for wp in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = wp['z']
            # Set orientation using yaw
            pose.pose.orientation.z = wp['yaw']  # Quaternion conversion needed
            goal.poses.append(pose)

        # Wait for action server
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        future.result()

def main(args=None):
    rclpy.init(args=args)
    follower = WaypointFollower()
    
    follower.send_waypoints('agbot/path_follow/paths/s_shape_waypoints.yaml')
    
    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
