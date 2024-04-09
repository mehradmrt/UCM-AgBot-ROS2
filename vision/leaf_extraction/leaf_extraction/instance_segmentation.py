import numpy as np
import open3d as o3d
import struct
import cv2
from PIL import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('instance_segmentation')
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)
        
        self.pose_array_publisher = self.create_publisher(PoseArray,'/target_leaves',
                                        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
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

        self.top_n = 10
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
        self.locations = self.transform_axes_and_calculate_rotation()
        self.publish_pose_array()
        print(self.locations)

        # self.plot_masked_points()
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
                # filtered_masked_points = self.filter_outliers_dbscan(masked_points)
                filtered_masked_points = masked_points
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

            if normal_vector[2] > 0:
                normal_vector = -normal_vector
            
            vectors.append(normal_vector)

        return vectors

    def axes_for_masks(self):
        axes = []

        for i in range(len(self.masks_xyzs)):

            ## first approach (closest point)
            # edge_points = self.find_edge_points(self.ordered_masks[i])
            # edge_xyz_points = self.points.reshape((self.height, self.width, 3))[edge_points[:, 0], edge_points[:, 1]]
            # distances = np.linalg.norm(edge_xyz_points - self.midpoints[i], axis=1)
            # closest_point = edge_xyz_points[np.argmin(distances)]
            # vector_to_midpoint = closest_point - self.midpoints[i]

            ## Second approach (stem from the lowest y value)
            mask_xyz_points = self.masks_xyzs[i]
            lowest_y_point = mask_xyz_points[np.argmin(mask_xyz_points[:, 1])]
            vector_to_midpoint = lowest_y_point - self.midpoints[i]

            stem_mid_axis = self.project_vector_onto_plane(vector_to_midpoint, self.normal_vectors[i])
            cross_axis = np.cross(self.normal_vectors[i], stem_mid_axis)
            axes.append([self.normal_vectors[i], stem_mid_axis, cross_axis]) 

            ## third approach (stem from the highest y value or the tip)
            # mask_xyz_points = self.masks_xyzs[i]
            # highest_y_point = mask_xyz_points[np.argmax(mask_xyz_points[:, 1])]
            # vector_to_midpoint = highest_y_point - self.midpoints[i]

            # tip_mid_axis = self.project_vector_onto_plane(vector_to_midpoint, self.normal_vectors[i])
    
            # cross_axis = np.cross(self.normal_vectors[i], tip_mid_axis)
            # axes.append([-self.normal_vectors[i], -tip_mid_axis, cross_axis]) 
            # # axes.append([-self.normal_vectors[i], -tip_mid_axis, cross_axis]) #rotat3 180 deg around Z to protect sensors on RG2

        return axes

    def find_edge_points(self, mask):
        edges = cv2.Canny(mask.astype(np.uint8) * 255, 100, 200)
        return np.argwhere(edges > 0)

    def project_vector_onto_plane(self, vector, normal_vector):
        projected_vector = vector - np.dot(vector, normal_vector) * normal_vector
        return projected_vector / np.linalg.norm(projected_vector)

    def transform_axes_and_calculate_rotation(self):
        transformed_elements = []
        for i, axis_set in enumerate(self.axes):
            ''' if sent to the gripper directly '''

            position = self.midpoints[i]
            transformed_p=( 
                            np.array([17.5, 124.33, -195.62])*0.001+  ### the vector that connects RG2 to camera
                            np.array([0.0, 0.0, -15.0])*0.001+ ### the gap between the new printed fingers and the old ones  
                            np.array([-position[0], -position[1], position[2]])
                            # np.array([0, 0, 230.0]) ### subtract the flange to endeffector vector for Moveit 
                            )
            
            axis1, axis2, axis3 = axis_set[0], axis_set[1], axis_set[2]

            transformed_axes = np.array([[-axis1[0], -axis1[1], axis1[2]],
                                         [-axis2[0], -axis2[1], axis2[2]],
                                         [-axis3[0], -axis3[1], axis3[2]]])

            rotmat = R.from_matrix(transformed_axes).inv()
            rotation_as_quat = rotmat.as_quat()
            transformed_elements.append(np.concatenate([transformed_p, rotation_as_quat]))

            ''' if sent to the "camera_depth_optical_frame" directly '''
            # position = self.midpoints[i]
            # transformed_p=(np.array([position[0], position[1], position[2]])) 
            # axis1, axis2, axis3 = axis_set[0], axis_set[1], axis_set[2]

            # transformed_axes = np.array([[axis1[0], axis1[1], axis1[2]],
            #                              [axis2[0], axis2[1], axis2[2]],
            #                              [axis3[0], axis3[1], axis3[2]]])
            # rotmat = R.from_matrix(transformed_axes).inv()
            # rotation_as_quat = rotmat.as_quat()
            # transformed_elements.append(np.concatenate([transformed_p, rotation_as_quat]))

        return np.array(transformed_elements)

    def publish_pose_array(self):
        pose_array_msg = PoseArray()

        for transformed_element in self.locations:
            pose_msg = Pose()
            pose_msg.position.x = transformed_element[0]
            pose_msg.position.y = transformed_element[1]
            pose_msg.position.z = transformed_element[2]
            pose_msg.orientation.x = transformed_element[3]
            pose_msg.orientation.y = transformed_element[4]
            pose_msg.orientation.z = transformed_element[5]
            pose_msg.orientation.w = transformed_element[6]

            pose_array_msg.poses.append(pose_msg)

        self.pose_array_publisher.publish(pose_array_msg)

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

            for axis_index, axis_color in enumerate(['black', 'green', 'blue']):  # X, Y, Z axes
                axis_vector = self.axes[i][axis_index]
                ax.quiver(central_point[0], central_point[1], central_point[2], 
                        axis_vector[0], axis_vector[1], axis_vector[2], 
                        length=0.1, color=axis_color, normalize=True)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=10, azim=270)
        plt.show()

    # def plotting_o3d(self):
    #     cloud = o3d.geometry.PointCloud()
    #     cloud.points = o3d.utility.Vector3dVector(self.points)
    #     # cloud.colors = o3d.utility.Vector3dVector(self.colors / 255.0) 

    #     xyz_reshaped = self.points.reshape((self.height, self.width, 3))
    #     filtered_xyz = xyz_reshaped[self.combined_masks.astype(bool)]
    #     filtered_cloud = o3d.geometry.PointCloud()
    #     filtered_cloud.points = o3d.utility.Vector3dVector(filtered_xyz)
    #     filtered_cloud.paint_uniform_color([0, 0, 0]) 

    #     coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
    #     vis = o3d.visualization.Visualizer()
    #     vis.create_window()
    #     vis.add_geometry(cloud)
    #     vis.add_geometry(filtered_cloud)
    #     vis.add_geometry(coordinate_frame)
    #     vis.run()
    #     vis.destroy_window()
        
    def plotting_o3d(self):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.points)
        cloud.colors = o3d.utility.Vector3dVector(self.colors / 255.0)

        xyz_reshaped = self.points.reshape((self.height, self.width, 3))
        filtered_xyz = xyz_reshaped[self.combined_masks.astype(bool)]
        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_xyz)
        filtered_cloud.paint_uniform_color([0, 1, 0])

        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(cloud)
        # vis.add_geometry(filtered_cloud)
        vis.add_geometry(coordinate_frame)

        num_masks = min(len(self.masks_xyzs), self.top_n)

        for i in range(num_masks):
            points = self.masks_xyzs[i]
            if points.size == 0 or self.midpoints[i] is None:
                continue

            central_point = self.midpoints[i]
            midpoint_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
            midpoint_sphere.translate(central_point)
            midpoint_sphere.paint_uniform_color([1, 0, 0])
            vis.add_geometry(midpoint_sphere)

            for axis_index, axis_color in enumerate([[1, 0, 0], [0, 1, 0], [0, 0, 1]]):  
                axis_vector = self.axes[i][axis_index] * 0.1  
                axis_line = o3d.geometry.LineSet()
                points = [central_point, central_point + axis_vector]
                lines = [[0, 1]]
                colors = [axis_color]  
                axis_line.points = o3d.utility.Vector3dVector(points)
                axis_line.lines = o3d.utility.Vector2iVector(lines)
                axis_line.colors = o3d.utility.Vector3dVector(colors)
                vis.add_geometry(axis_line)

        vis.run()
        vis.destroy_window()

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin_once(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()