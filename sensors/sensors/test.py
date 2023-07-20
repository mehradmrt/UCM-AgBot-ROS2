import serial
import pynmea2

port = '/dev/ttyACM0'
baud = 115200

ser = serial.Serial(port, \
        baudrate=baud,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=1)

while True:
        data = ser.readline()
        # print(ser)
        # print(data)
        if data:
                line = data.decode("utf-8").rstrip()
                if line.startswith('$GNGGA'):
                        msg = pynmea2.parse(line)
                        print(msg.latitude)
