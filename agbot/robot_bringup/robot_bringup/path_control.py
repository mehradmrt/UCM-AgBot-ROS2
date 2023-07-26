import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray


class PathControl(Node):
    def __init__(self):
        super().__init__('path_controller')
        # minimal value in meter on front & left & right
        self.minimal_value_front: float = 0.5
        self.minimal_value_left: float = 0.01
        self.minimal_value_right: float = 0.01
        
        self.ratio_left: float = self.minimal_value_left / 2.5
        self.ratio_right: float = self.minimal_value_right / 2.5

        # motors speed
        self.cruise_speed: int = 50

        self.motor_speed = [int] * 2
        # left
        self.motor_speed[0] = 0
        # right
        self.motor_speed[1] = 0

        # publishing part
        self.publisher_ = self.create_publisher(Int16MultiArray, 'motor_command', 10)

        # subscribing part
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
    
        print("reading data..")

        # to change motor speed the priority is FRONT > RIGHT > LEFT > CRUISE / gps data
        self.motor_speed[0] = self.cruise_speed
        self.motor_speed[1] = self.cruise_speed
        
        # follow gps data
 
        # LEFT, motor right speed depend on the closest distance between something
        # get the closest distance between something
        # between 20° to 60° so 20*2=40+1 to 60*2=120
        tab_left = []
        for i in range(41, 120):
            tab_left.append(msg.ranges[i])
        min_left = min(tab_left)

        print('Minimal value on the left :%f' % min_left)
        # speed change only bellow minimal value
        if min_left < self.minimal_value_left:
            # 100 is the regular speed
            # motor left
            self.motor_speed[0] = self.cruise_speed
            # motor right speed, 100 when minimal value , 50 at 60% of minimal value, 0 at 20% of minimal value, below it stop
            # default example for something at 3 meter speed = (100/2) * (3/2) - (100/4) = 50
            self.motor_speed[1] = (self.cruise_speed / 2) * (min_left /self.ratio_left) - (self.cruise_speed / 4)

        # RIGHT, motor right speed depend on the closest distance between something
        # get the closest distance between something
        # between -60° to -20° so 1080-60*2=960 to 1080-20*2=1040 -1
        tab_right = []
        for i in range(960, 1040):
            tab_right.append(msg.ranges[i])
        min_right = min(tab_right)

        print('Minimal value on the right :%f' % min_right)
        # speed change only bellow minimal value
        if min_right < self.minimal_value_right:
            # 100 is the regular speed
            # motor left speed, 100 when minimal value , 50 at 60% of minimal value, 0 at 20% of minimal value, below it stop
            # default example for something at 3 meter speed = (100/2) * (3/2) - (100/4) = 50
            self.motor_speed[0] = (self.cruise_speed / 2) * (min_left /self.ratio_left) - (self.cruise_speed / 4)
            # motor right
            self.motor_speed[1] = self.cruise_speed

        # FRONT < 1 meter => stop the motor
        # -20° to 20° => 1080-20*2 = 1040 to 20*2 = 40
        # so 1040 to 1080 and 0 to 40
        for i in range(1040, 1080):
            if msg.ranges[i] < self.minimal_value_front:
                self.motor_speed[0] = 0
                self.motor_speed[1] = 0
                break

        for i in range(0, 40):
            if msg.ranges[i] < self.minimal_value_front:
                self.motor_speed[0] = 0
                self.motor_speed[1] = 0
                break
                

        self.data_sending()

    def data_sending(self):
        msg = Int16MultiArray()
        msg.data = [int(self.motor_speed[0]), int(self.motor_speed[1])]

        self.publisher_.publish(msg)
        print('Publishing data: left:%s right:%s' % (msg.data[0], msg.data[1]))


def main(args=None):
    rclpy.init(args=args)
    path_controller = PathControl()
    print("Path_control launch")
    rclpy.spin(path_controller)

    path_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
