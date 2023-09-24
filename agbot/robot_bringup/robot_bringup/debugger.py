
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

class IMUPlotter(Node):
    def __init__(self):
        super().__init__('imu_plotter')
        self.subscription = self.create_subscription(Imu, 'vectornav/imu_uncompensated', self.imu_callback, 10)
        
        plt.ion()
        self.fig, self.ax = plt.subplots(9, 1, figsize=(8, 15))
        
        # Titles for each subplot
        titles = ['Roll', 'Pitch', 'Yaw', 
                  'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z', 
                  'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z']
        
        for i, title in enumerate(titles):
            self.ax[i].set_title(title)
        
        # Data storage
        self.roll_data, self.pitch_data, self.yaw_data = [], [], []
        self.ang_vel_x_data, self.ang_vel_y_data, self.ang_vel_z_data = [], [], []
        self.lin_acc_x_data, self.lin_acc_y_data, self.lin_acc_z_data = [], [], []

    def imu_callback(self, msg):
        q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        euler = q.yaw_pitch_roll
        yaw, pitch, roll = float(euler[0]), float(euler[1]), float(euler[2])
        ang_vel_x, ang_vel_y, ang_vel_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        lin_acc_x, lin_acc_y, lin_acc_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        # Append data
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        self.yaw_data.append(yaw)
        self.ang_vel_x_data.append(ang_vel_x)
        self.ang_vel_y_data.append(ang_vel_y)
        self.ang_vel_z_data.append(ang_vel_z)
        self.lin_acc_x_data.append(lin_acc_x)
        self.lin_acc_y_data.append(lin_acc_y)
        self.lin_acc_z_data.append(lin_acc_z)

        # Update plots
        data_lists = [ self.yaw_data, self.pitch_data, self.roll_data, 
                      self.ang_vel_x_data, self.ang_vel_y_data, self.ang_vel_z_data, 
                      self.lin_acc_x_data, self.lin_acc_y_data, self.lin_acc_z_data]
        
        for i, data in enumerate(data_lists):
            self.ax[i].clear()
            self.ax[i].plot(data)
            self.ax[i].set_title(self.ax[i].get_title())  # Reset title after clearing
            self.ax[i].grid(True) 

        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)

    imu_plotter = IMUPlotter()

    rclpy.spin(imu_plotter)

    imu_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()