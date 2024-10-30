import serial
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

class Axes:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def update(self, x: float = None, y: float = None, z: float = None):
        self.x = float(x) if x else self.x
        self.y = float(y) if y else self.y
        self.z = float(z) if z else self.z


ser = serial.Serial("COM3", 38400, timeout=0.1)
x = 0
fig, ax = plt.subplots()
ax.set_xlim(0, 20)
ax.set_ylim(-180, 180)

x_data = []
y_data = []
line, = ax.plot(x_data, y_data)


def read_data():
    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")
    return line

    # if len(angles) == 6:
    #     obj0.update(*angles[:3])
    #     obj1.update(*angles[3:])


def get_position(imuNum: int):
        values = [float(i.strip()) for i in read_data().decode('utf-8').split(',') if i]
        if len(values) == 6:
            imu_1 = Axes(*values[:3])
            imu_2 = Axes(*values[3:])

            print("IMU 1")
            print(f"roll (x): {imu_1.x}, pitch (y): {imu_1.y}, yaw (z): {imu_1.z}")
            print("IMU 2")
            print(f"roll (x): {imu_2.x}, pitch (y): {imu_2.y}, yaw (z): {imu_2.z}")
            print()

            # return values[:3] if imuNum == 1 else values[3:]
            return imu_1.x if imuNum == 1 else imu_2.x  # just x for now
        else:
            return 0

def update(frame):
    # Assuming data is a single float value for y-axis
    global x
    precision = 20
    y = get_position(1)
    print(y)
    x_data.append(x)
    y_data.append(y)

    # Keep only the last 100 points to avoid memory issues
    x_data[:] = x_data[-precision:]
    y_data[:] = y_data[-precision:]

    # Update plot data
    line.set_data(x_data, y_data)
    ax.set_xlim(max(0, x - precision), x)  # Slide the x-axis
    # ax.set_ylim(min(-2, min(y_data)), max(2, max(y_data)))  # stretch y axis
    # print(x_data)
    # print(y_data)
    x+=1
    return line


ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
plt.show()
#
# while(1):
#     get_position(1)
