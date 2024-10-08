import socket
import time
import struct

class RobotCommands:
    def __init__(self):
        self.server_ip = "192.168.1.7"
        self.port = 20001
        self.socket = None
        self.lift_speed = 0
        self.lift_ctrl = 0
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.motor_rotation_ctrl = 0

        # Data packet (13 bytes) - initialize with default values
        self.data = bytearray([0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38])

    def is_connected(self):
        return self.socket is not None

    def connection(self):
        while not self.is_connected():
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.port))
                print("Connection established")
            except socket.error as e:
                print(f"Failed to connect: {e}. Retrying...")
                time.sleep(1)

    def set_lift_speed(self, speed):
        self.data[6] = speed

    def set_lift_ctrl(self, ctrl):
        self.data[7] = ctrl

    def set_left_speed(self, speed):
        self.data[9] = speed

    def set_right_speed(self, speed):
        self.data[10] = speed

    def set_direction_ctrl(self, state):
        self.data[11] = state

    def set_neutral(self):
        self.data = bytearray([0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38])

    def rotation_check(self):
        return self.data[11]

    def sending_data(self):
        while not self.is_connected():
            print("Connection lost. Attempting to reconnect...")
            self.connection()
        
        try:
            self.socket.sendall(self.data)
            print(f"Sending data: {list(self.data)}")
        except socket.error as e:
            print(f"Failed to send data: {e}")
            self.socket.close()
            self.socket = None  # Mark as disconnected
