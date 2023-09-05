#%%
import serial

# open serial port
ser = serial.Serial('/dev/ttyACM1', 115200)  # replace 'COM3' with your serial port

try:
    while True:
        # read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        print(line)

except KeyboardInterrupt:
    pass
finally:
    ser.close()

# %%
