import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from roboflow import Roboflow
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')

        self.depth_subscription = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()

        
        self.rf = Roboflow(api_key="UqYdMLWWT9ibnLSz86YT")
        self.model = self.rf.workspace().project("actualdetectingleaves").version(2).model
        self.processed = False  # To ensure we process only once

    def rgb_callback(self, msg):
        if self.processed:
            return
        
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV format: {e}")
            return

        image_path = "/tmp/temp_rgb.jpg"
        try:
            cv2.imwrite(image_path, self.rgb_image)
        except Exception as e:
            self.get_logger().error(f"Error writing image to disk: {e}")
            return

        try:
            prediction = self.model.predict(image_path).json()
            sorted_predictions = self.get_best_leaf_prediction(prediction)
            best_prediction = sorted_predictions[2]  # Take the highest confidence instance
        except Exception as e:
            self.get_logger().error(f"Error getting predictions from Roboflow: {e}")
            return
        
        contour_points_list = [[int(point["x"]), int(point["y"])] for point in best_prediction["points"]]
        mask = np.zeros(self.rgb_image.shape[:2], dtype=np.uint8)

        # Draw the contour
        cv2.drawContours(mask, [np.array(contour_points_list)], 0, (255), thickness=-1)  # The -1 thickness will fill the contour

        # Apply mask on RGB image to get instance image
        instance_img = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=mask)
        cv2.imwrite("/tmp/extracted_instance.jpg", instance_img)

        if best_prediction:
            points, colors = [], []
            for y in range(self.rgb_image.shape[0]):
                for x in range(self.rgb_image.shape[1]):
                    if mask[y, x] > 0:
                        depth = self.depth_image[y, x]
                        if depth > 0:
                            points.append([x, y, depth])
                            # colors.append(self.rgb_image[y, x])

            x = [pt[0] for pt in points]
            y = [pt[1] for pt in points]
            z = [pt[2] for pt in points]

            # Create a new figure
            fig = plt.figure()

            # Add 3d subplot
            # The '111' means 1x1 grid, first subplot, similar to MATLAB-style
            ax = fig.add_subplot(111, projection='3d')

            # Scatter plot
            ax.scatter(x, y, z, c='r', marker='o')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Depth')

            # Save the figure as an image
            plt.savefig("/tmp/3d_scatter_plot.png")

            # Show the plot in a window
            plt.show()




        self.processed = True  # Mark that we've processed a frame

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Error converting depth ROS Image to OpenCV format: {e}")

    def get_best_leaf_prediction(self, prediction_data):
        good_leaf_predictions = [p for p in prediction_data["predictions"] if p["class"] == "good leaf"]
        sorted_predictions = sorted(good_leaf_predictions, key=lambda x: -x["confidence"])  # sort in descending order
        return sorted_predictions

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
