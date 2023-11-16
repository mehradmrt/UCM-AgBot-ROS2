import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO
import sensor_msgs_py.point_cloud2 as pc2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/arya/instance_segmentation/best.pt')
        self.rgb_image = None
        self.point_cloud = None
        self.mask = None
        self.processed = False

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
            cv2.imwrite("/tmp/new_rgb.jpg", bgr_image)
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")
            return

        results = self.model(['/tmp/new_rgb.jpg'], imgsz=1280, save=True)
        for result in results:
            num = 3
            best_mask = result.masks.masks[num]
            best_mask_np = best_mask.cpu().numpy()
            self.mask = cv2.resize(best_mask_np, (1280, 720), interpolation=cv2.INTER_NEAREST)
            print(self.mask.shape)

    def point_cloud_callback(self, msg):
        if self.processed or self.mask is None:
            return

        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)))
        
        # Extract XYZ values
        xyz = points[:, :3]

        # Reshape XYZ to match the mask's shape
        xyz_reshaped = xyz.reshape((720, 1280, 3))
        rgb_image_reshaped = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB).reshape((720, 1280, 3))

        square_size = 300

        start_row = xyz_reshaped.shape[0] - square_size  # Starting row index
        start_col = xyz_reshaped.shape[1] - square_size  # Starting column index
        bottom_right_square = xyz_reshaped[start_row:, start_col:]
        bottom_right_square_points = bottom_right_square.reshape(-1, 3)
        square_cloud = o3d.geometry.PointCloud()
        square_cloud.points = o3d.utility.Vector3dVector(bottom_right_square_points)
        square_cloud.paint_uniform_color([0, 0, 0])  # Red color for the square points

        # top_left_square = xyz_reshaped[0:square_size, 0:square_size]
        # top_left_square_points = top_left_square.reshape(-1, 3)
        # print("Shape of xyz_reshaped:", xyz_reshaped.shape)
        # print("Shape of top_left_square:", top_left_square.shape)
        # print("Shape of top_left_square_points:", top_left_square_points.shape)

        # square_cloud = o3d.geometry.PointCloud()
        # square_cloud.points = o3d.utility.Vector3dVector(top_left_square_points)
        # square_cloud.paint_uniform_color([0, 0, 0])  # Red color for the square points

        # Create a full point cloud
        full_cloud = o3d.geometry.PointCloud()
        full_cloud.points = o3d.utility.Vector3dVector(xyz)
        full_cloud.colors = o3d.utility.Vector3dVector(rgb_image_reshaped.reshape(-1, 3) / 255.0)
        # full_cloud.paint_uniform_color([0.5, 0.5, 0.5])

        # Filtered points (using the mask)
        filtered_xyz = xyz_reshaped[self.mask > 0]
        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_xyz)
        filtered_cloud.paint_uniform_color([0, 0, 0])  # Black color for filtered points

        # Visualization
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(full_cloud)
        vis.add_geometry(filtered_cloud)
        vis.add_geometry(square_cloud)
        vis.add_geometry(coordinate_frame)
        vis.run()
        vis.destroy_window()

        self.processed = True


def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()