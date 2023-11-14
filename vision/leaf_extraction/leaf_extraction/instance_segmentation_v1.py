import cv2
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')

        self.depth_subscription = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        
        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/arya/instance segmentation/best.pt')
        self.processed = False

    def rgb_callback(self, msg):
        if self.processed:
            return

        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
            cv2.imwrite("/tmp/new_rgb.jpg", bgr_image)
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")
            return

        results = self.model(['/tmp/new_rgb.jpg'], imgsz=1280, save=True)
        all_points=[]

        for result in results:
            for num in range(4):    
                best_mask = result.masks.masks[num]
                # print(best_mask.shape)
                
                best_mask_np = best_mask.cpu().numpy()
                resized_mask = cv2.resize(best_mask_np, (1280, 720), interpolation=cv2.INTER_NEAREST)

                try:
                    depths = [self.depth_image[y, x]
                                for y, x in zip(*np.where(resized_mask))]
                    points = [(x, y, depth)
                                for x, y, depth in zip(*np.where(resized_mask), depths)]
                    all_points.extend(points)
                except Exception as e:
                    self.get_logger().error(f"issue with depth post-processing: {e}")
            self.plot_3d_scatter(all_points)

        self.processed = True

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Error with receiving depth image: {e}")

    def plot_3d_scatter(self, points):
        x = [pt[0] for pt in points]
        y = [pt[1] for pt in points]
        z = [pt[2] for pt in points]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, c='r', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Depth')

        plt.savefig("/tmp/3d_scatter_plot.png")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
