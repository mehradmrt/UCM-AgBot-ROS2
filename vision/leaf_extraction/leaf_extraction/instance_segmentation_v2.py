import itertools
import cv2
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs_py import point_cloud2


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')

        # Initialize the subscribers
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)

        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Initialize image and point cloud placeholders
        self.rgb_image = None
        self.point_cloud = None

        # Load YOLO model
        self.model = YOLO('/home/arya/instance_segmentation/best.pt')
        self.processed = False

    def rgb_callback(self, msg):
        if self.processed or self.point_cloud is None:
            return

        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
            cv2.imwrite("/tmp/new_rgb.jpg", bgr_image)
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")
            return

        results = self.model(['/tmp/new_rgb.jpg'], imgsz=1280, save=True)
        all_points = []

        for result in results:
            num = 1
            best_mask = result.masks.masks[num]
            best_mask_np = best_mask.cpu().numpy()
            resized_mask = cv2.resize(best_mask_np, (1280, 720), interpolation=cv2.INTER_NEAREST)

            # Convert the PointCloud2 data to an array
            point_array = np.asarray(list(point_cloud2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=False)))

            # Reshape the point array to match the dimensions of the mask
            point_array_reshaped = point_array.reshape((self.point_cloud.height, self.point_cloud.width, 3))

            # Iterate over the mask and get the corresponding points from the reshaped array
            mask_indices = np.where(resized_mask)
            for y, x in zip(*mask_indices):
                point = point_array_reshaped[y, x]
                if not np.isnan(point).any():
                    all_points.append(point)

        self.plot_3d_scatter(all_points)
        self.processed = True

    def point_cloud_callback(self, msg):
        self.point_cloud = msg

    def plot_3d_scatter(self, points):
        if not points:
            return

        x = [pt[0] for pt in points]
        y = [pt[1] for pt in points]
        z = [pt[2] for pt in points]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, c='r', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')  # Changed from 'Depth' to 'Z' for 3D consistency

        plt.savefig("/tmp/3d_scatter_plot.png")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
