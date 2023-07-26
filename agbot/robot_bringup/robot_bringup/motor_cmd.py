import socket

class RobotControl:
    S = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        self.server = '192.168.0.7'
        self.port = 20001
        
        self.lift_speed: int = 0  # 0 -> stop, 250 -> run
        self.lift_ctrl: int = 0  # 0 -> nothing, 1 -> down, 2 -> up
        self.left_motor_speed: int = 0  # 0
        self.right_motor_speed: int = 0  # 0
        self.motor_left_ctrl: int = 0  # 0 left cw, 1 = left ccw
        self.motor_right_ctrl: int = 0  # 0 right cw, 2 = right ccw
        self.motor_ctrl_state: int = 0
        self.motor_rotation_ctrl: int = 0 # fwd:0 , cw:1 , ccw:2, bwd:3

        self.data = [0x08, 0x00, 0x00, 0x00, 0x12, 0x00, self.lift_speed, self.lift_ctrl, 0x00,
                abs(self.left_motor_speed), abs(self.right_motor_speed),
                self.motor_rotation_ctrl,
                0x38]

        # self.data = [0x08, 0x00, 0x00, 0x00, 0x12, 0x00, self.lift_speed, self.lift_ctrl, 0x00,
        #              abs(self.left_motor_speed), abs(self.right_motor_speed),
        #              self.motor_left_ctrl + self.motor_right_ctrl * 2,
        #              0x38]

    def connection(self):
        # Connection
        server= self.server
        port = self.port
        addr = (server, port)
        self.S.connect(addr)
        print("Connection Done\n")

    def set_lift_speed(self, speed):
        self.data[6] = speed

    def set_lift_ctrl(self, ctrl):
        self.data[7] = ctrl

    def set_left_speed(self, speed):
        self.data[9] = speed

    def set_right_speed(self, speed):
        self.data[10] = speed
    
    def set_rotation_ctrl(self, value):
        self.data[11] = value

    def set_motor_ctrl(self, left_motor, right_motor):
        ###############
        # this part is to pass through the issue when we want to change the motor direction
        # left_motor and right_motor : -180 ~ 180, 0/180 =>  forward, -180/0 => backward
        ###############
        # comparing current command with new command
        if self.right_motor_speed >= 0 & right_motor < 0:
            self.motor_ctrl_state = 0

        if self.right_motor_speed <= 0 & right_motor > 0:
            self.motor_ctrl_state = 0

        if self.left_motor_speed >= 0 & left_motor < 0:
            self.motor_ctrl_state = 0

        if self.left_motor_speed <= 0 & left_motor > 0:
            self.motor_ctrl_state = 0

        # current command = new command
        self.right_motor_speed = right_motor
        self.left_motor_speed = left_motor

        if self.motor_ctrl_state == 0:
            # stop the motors
            self.data[9] = 60
            self.data[10] = 60

        if self.motor_ctrl_state == 1:
            # reset direction
            self.data[11] = 0

        if self.motor_ctrl_state == 2:
            # change direction

            # matching data to control motor and speed data
            # between 0 and 180
            if self.left_motor_speed >= 0:
                self.motor_left_ctrl = 1
            # between -180 and -0.1
            else:
                self.motor_left_ctrl = 0

            if self.right_motor_speed >= 0:
                self.motor_right_ctrl = 2
            else:
                self.motor_right_ctrl = 0

            # implementing the data
            self.data[11] = self.motor_left_ctrl + self.motor_right_ctrl

        # default case (normal)
        if self.motor_ctrl_state > 4:
            # we have to change direction 3 times to make it accurate
            self.data[9] = abs(self.left_motor_speed) + 60
            self.data[10] = abs(self.right_motor_speed) + 60

        if self.left_motor_speed == 0:
            self.data[9] = 0
        if self.right_motor_speed == 0:
            self.data[9] = 0
        # increase case
        if self.motor_ctrl_state < 10:
            self.motor_ctrl_state += 1

    def sending_data(self):
        print(self.data)
        self.S.send(bytes(self.data))
