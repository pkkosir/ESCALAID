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
ax.set_xlim(0, 100)
ax.set_ylim(-180, 180)

x_data = []
roll_data, pitch_data = [], []
roll, = ax.plot(x_data, roll_data, animated=True)
pitch, = ax.plot(x_data, pitch_data, animated=True)


def read_data():
    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")
    return line


def get_position():
    #values = [float(i.strip()) for i in read_data().decode('utf-8').split(',') if i]
    data = read_data().decode("utf-8").split(",")
    if len(data) == 6:

        # print("IMU 1")
        # print(f"roll (x): {imu_1.x}, pitch (y): {imu_1.y}, yaw (z): {imu_1.z}")
        # print("IMU 2")
        # print(f"roll (x): {imu_2.x}, pitch (y): {imu_2.y}, yaw (z): {imu_2.z}")
        # print()

        # return values[:3] if imuNum == 1 else values[3:]
        return float(data[0]), float(data[1])  # just x for now
    else:
        return 0


def update(frame):
    # Assuming data is a single float value for y-axis
    global x
    precision = 20
    y_roll, y_pitch = get_position()
    print(f"roll {roll}, pitch {pitch}")
    x_data.append(x)
    roll_data.append(y_roll)
    pitch_data.append(y_pitch)

    # Keep only the last 100 points to avoid memory issues
    x_data[:] = x_data[-precision:]
    roll_data[:] = roll_data[-precision:]
    pitch_data[:] = pitch_data[-precision:]

    # Update plot data
    roll.set_data(x_data, roll_data)
    pitch.set_data(x_data, pitch_data)
    ax.set_xlim(max(0, x - precision), x)  # Slide the x-axis
    # print(x_data)
    # print(y_data)
    x+=1
    return roll, pitch


ani = FuncAnimation(fig, update, init_func=lambda: (roll, pitch), interval=100, blit=True, cache_frame_data=True)
plt.show()
#
# while(1):
#     get_position(1)
