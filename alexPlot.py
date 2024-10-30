import serial
import time

class Axes:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def update(self, x: float = None, y: float = None, z: float = None):
        self.x = float(x) if x else self.x
        self.y = float(y) if y else self.y
        self.z = float(z) if z else self.z


ser = serial.Serial("COM3", 38400, timeout=3)
yaw_mode = False
obj0, obj1 = Axes(), Axes()


def read_data():
    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")
    return angles

    # if len(angles) == 6:
    #     obj0.update(*angles[:3])
    #     obj1.update(*angles[3:])

while(1):
    time.sleep(0.2)
    print(read_data())