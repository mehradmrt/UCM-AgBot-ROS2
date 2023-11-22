import numpy as np
import open3d as o3d
import struct
import cv2
from PIL import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)

        self.bridge = CvBridge()
        self.model = YOLO('/home/arya/instance_segmentation/best.pt')
        self.conf_cutoff = 0.7

        self.colors = None
        self.points = None
        self.point_cloud = None

        self.width = None
        self.height = None

        self.combined_masks = None
        self.ordered_masks = None
        self.confs = None
        self.masks_xyzs = None
        self.midpoints = None
        self.normal_vectors = None
        self.axes = None

        self.top_n = 5
        self.dbscan_eps = 0.01
        self.dbscan_ms = 10

        self.processed = False

    def point_cloud_callback(self, msg):
        if self.processed:
            return

        self.points, self.colors = self.extract_points_and_colors(msg)
        self.combined_masks, self.ordered_masks, self.confs = self.extract_masks()
        self.masks_xyzs = self.extract_masks_xyzs()
        self.midpoints = self.extract_midpoints()
        self.normal_vectors = self.fit_plane_and_find_normal()
        self.axes = self.axes_for_masks()
        print(self.axes)

        self.plot_masked_points()
        self.plotting_o3d()
        

        self.processed = True

    def extract_points_and_colors(self, cloud_msg):

        cloud_data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        self.height, self.width = cloud_msg.height, cloud_msg.width

        points = np.zeros((self.height * self.width, 3), dtype=np.float32)
        colors = np.zeros((self.height * self.width, 3), dtype=np.uint8)

        for i in range(self.height * self.width):
            point_start = i * cloud_msg.point_step
            x, y, z = struct.unpack_from('fff', cloud_data, offset=point_start)
            points[i] = [x, y, z]

            rgb_packed = struct.unpack_from('I', cloud_data, offset=point_start + 16)[0]
            colors[i] = [(rgb_packed >> 16) & 0xFF, (rgb_packed >> 8) & 0xFF, rgb_packed & 0xFF]

        return points, colors

    def extract_masks(self):
        # height, width = 720, 1280 
        image_array = self.colors.reshape((self.height, self.width, 3)).astype(np.uint8)
        image = Image.fromarray(image_array, 'RGB')

        open_cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        cv2.imwrite("/home/arya/AgBot_reps/UCM-AgBot-ROS2/src/vision/leaf_extraction/segmentation_model/pc_rgb.jpg", open_cv_image)
        results = self.model([open_cv_image], imgsz=self.width, save=True, conf=self.conf_cutoff)

        combined_masks = np.zeros((self.height, self.width), dtype=np.uint8)
        ordered_masks = []

        for result in results:
            confs = result.boxes.conf.numpy()
            for i in range(result.__len__()):  
                mask_i = result.masks.masks[i].numpy()
                resized_mask = cv2.resize(mask_i, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                ordered_masks.append(resized_mask)
                combined_masks = np.logical_or(combined_masks, resized_mask).astype(np.uint8)

        return combined_masks, ordered_masks, confs


    def extract_masks_xyzs(self):
        masks_xyzs = []
        xyz_reshaped = self.points.reshape((self.height, self.width, 3))

        for mask in self.ordered_masks:
            try:
                mask_boolean = mask.astype(bool)
                masked_points = xyz_reshaped[mask_boolean]
                filtered_masked_points = self.filter_outliers_dbscan(masked_points)
                depth_filtered_points = self.filter_points_by_depth(filtered_masked_points)
                masks_xyzs.append(depth_filtered_points)
            except Exception as e:
                self.get_logger().error('Error in extract_masks_xyzs function: {}'.format(str(e)))

        return masks_xyzs

    def filter_outliers_dbscan(self, points):
        eps=self.dbscan_eps
        min_samples=self.dbscan_ms

        # if len(points) < min_samples:
        #     return points
            
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_
        filtered_masked_points = points[labels != -1]

        return filtered_masked_points

    def filter_points_by_depth(self, points, Z_threshold=0.1):
        filtered_points = points[points[:, 2] > Z_threshold]
        return filtered_points


    def extract_midpoints(self):
        midpoints = []
        for points in self.masks_xyzs:

            median_point = np.median(points, axis=0)
            distances = np.linalg.norm(points - median_point, axis=1)
            closest_point_index = np.argmin(distances)
            central_point = points[closest_point_index]

            midpoints.append(central_point)

        return midpoints

    def fit_plane_and_find_normal(self):
        
        vectors = []
        for points in self.masks_xyzs:

            pca = PCA(n_components=3)
            pca.fit(points)
            normal_vector = pca.components_[-1]

            if normal_vector[1] > 0:
                normal_vector = -normal_vector
            
            vectors.append(normal_vector)

        return vectors

    def axes_for_masks(self):
        axes = []

        for i in range(len(self.masks_xyzs)):

            # Find edge points
            edge_points = self.find_edge_points(self.ordered_masks[i])
            # Convert edge points to XYZ coordinates
            edge_xyz_points = self.points.reshape((self.height, self.width, 3))[edge_points[:, 0], edge_points[:, 1]]

            # Find the closest edge point to the midpoint
            distances = np.linalg.norm(edge_xyz_points - self.midpoints[i], axis=1)
            closest_point = edge_xyz_points[np.argmin(distances)]

            # Vector from midpoint to closest edge point
            vector_to_midpoint = self.midpoints[i] - closest_point

            # Project this vector onto the plane to get X-axis
            x_axis = self.project_vector_onto_plane(vector_to_midpoint, self.normal_vectors[i])
            
            # Calculate Y-axis by crossing X and Z (normal) axes
            y_axis = np.cross(self.normal_vectors[i], x_axis)

            axes.append([x_axis, y_axis, self.normal_vectors[i]])

        return axes

    def find_edge_points(self, mask):
        edges = cv2.Canny(mask.astype(np.uint8) * 255, 100, 200)
        return np.argwhere(edges > 0)

    def project_vector_onto_plane(self, vector, normal_vector):
        projected_vector = vector - np.dot(vector, normal_vector) * normal_vector
        return projected_vector / np.linalg.norm(projected_vector)


    def plot_masked_points(self):
        num_masks = min(len(self.masks_xyzs), self.top_n)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        colors = plt.cm.get_cmap('hsv', num_masks)

        for i in range(num_masks):
            points = self.masks_xyzs[i]
            if points.size == 0 or self.midpoints[i] is None:
                continue

            central_point = self.midpoints[i]
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=colors(i), s=1)
            ax.scatter(central_point[0], central_point[1], central_point[2], color='black', s=60)

            for axis_index, axis_color in enumerate(['red', 'green', 'blue']):  # X, Y, Z axes
                axis_vector = self.axes[i][axis_index]
                ax.quiver(central_point[0], central_point[1], central_point[2], 
                        axis_vector[0], axis_vector[1], axis_vector[2], 
                        length=0.1, color=axis_color, normalize=True)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=10, azim=120)
        plt.show()



    # def plot_masked_points_with_centers(self):
    #     num_masks = min(len(self.masks_xyzs), self.top_n)
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     colors = plt.cm.get_cmap('hsv', num_masks)

    #     for i in range(num_masks):
    #         points = self.masks_xyzs[i]
    #         if points.size == 0:
    #             continue
            
    #         median_point = np.median(points, axis=0)
    #         distances = np.linalg.norm(points - median_point, axis=1)
    #         closest_point_index = np.argmin(distances)
    #         central_point = points[closest_point_index]

    #         ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=colors(i), s=1)
    #         ax.scatter(central_point[0], central_point[1], central_point[2], color='black', s=60)

    #         normal_vector = self.normal_vectors[i]
    #         vector_start = central_point
    #         vector_end = central_point + normal_vector * 0.1  
    #         ax.quiver(vector_start[0], vector_start[1], vector_start[2], 
    #                 normal_vector[0], normal_vector[1], normal_vector[2], 
    #                 length=0.1, color='black', normalize=True)

    #     ax.set_xlabel('X')
    #     ax.set_ylabel('Y')
    #     ax.set_zlabel('Z')
    #     ax.view_init(elev=10, azim=120)
    #     plt.show()


    def plotting_o3d(self):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.points)
        # cloud.colors = o3d.utility.Vector3dVector(self.colors / 255.0) 

        xyz_reshaped = self.points.reshape((self.height, self.width, 3))
        filtered_xyz = xyz_reshaped[self.combined_masks.astype(bool)]
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




def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()