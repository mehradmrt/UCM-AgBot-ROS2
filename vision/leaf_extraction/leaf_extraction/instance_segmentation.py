import numpy as np
import open3d as o3d
import struct
import cv2
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/arya/instance_segmentation/best.pt')
        self.colors = None
        self.points = None
        self.point_cloud = None
        self.mask = None
        self.processed = False

    def point_cloud_callback(self, msg):
        if self.processed:
            return

        self.points, self.colors = self.extract_points_and_colors(msg)
        self.mask = self.extract_masks()

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.points)
        cloud.colors = o3d.utility.Vector3dVector(self.colors / 255.0) 

        xyz_reshaped = self.points.reshape((720, 1280, 3))
        filtered_xyz = xyz_reshaped[self.mask > 0]
        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_xyz)
        filtered_cloud.paint_uniform_color([1, 0, 0]) 

        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(cloud)
        vis.add_geometry(filtered_cloud)
        vis.add_geometry(coordinate_frame)
        vis.run()
        vis.destroy_window()

        self.processed = True

    def extract_masks(self):
        height, width = 720, 1280 
        image_array = self.colors.reshape((height, width, 3)).astype(np.uint8)
        image = Image.fromarray(image_array, 'RGB')

        open_cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        cv2.imwrite("/home/arya/AgBot_reps/UCM-AgBot-ROS2/src/vision/leaf_extraction/segmentation_model/pc_rgb.jpg", open_cv_image)
        results = self.model([open_cv_image], imgsz=1280, save=True)

        combined_mask = np.zeros((height, width), dtype=np.uint8)

        for result in results:
            for num in range(0,2):  # Combines masks for num 0, 1, and 2
                if num < len(result.masks.masks):
                    best_mask = result.masks.masks[num]
                    best_mask_np = best_mask.cpu().numpy()
                    resized_mask = cv2.resize(best_mask_np, (width, height), interpolation=cv2.INTER_NEAREST)
                    combined_mask = np.logical_or(combined_mask, resized_mask > 0).astype(np.uint8)

        return combined_mask

    def extract_points_and_colors(self, cloud_msg):
        cloud_data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        points = np.zeros((cloud_msg.width * cloud_msg.height, 3), dtype=np.float32)
        colors = np.zeros((cloud_msg.width * cloud_msg.height, 3), dtype=np.uint8)

        for i in range(cloud_msg.width * cloud_msg.height):
            point_start = i * cloud_msg.point_step
            x, y, z = struct.unpack_from('fff', cloud_data, offset=point_start)
            points[i] = [x, y, z]

            rgb_packed = struct.unpack_from('I', cloud_data, offset=point_start + 16)[0]
            colors[i] = [(rgb_packed >> 16) & 0xFF, (rgb_packed >> 8) & 0xFF, rgb_packed & 0xFF]

        return points, colors



def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
