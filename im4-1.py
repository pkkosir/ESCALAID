from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint
import serial

RESOLUTION = 100

class IMU:
    def __init__(self):
        self.pitch = [0 for _ in range(RESOLUTION)]
        self.roll = [0 for _ in range(RESOLUTION)]
        self.yaw = [0 for _ in range(RESOLUTION)]
        self._axes = [self.pitch, self.roll, self.yaw]

    def update(self, x: float, y: float, z: float):
        for axis, point in zip(self._axes, [x, y, z]):
            axis.pop(0)
            axis.append(point if point is not None else axis[-1])

        # self.pitch = self.pitch[1:]
        # self.roll = self.roll[1:]
        # self.yaw = self.yaw[1:]
        #
        # self.pitch.append(float(x) if x is not None else self.pitch[-1])
        # self.roll.append(float(y) if y is not None else self.roll[-1])
        # self.yaw.append(float(z) if z is not None else self.yaw[-1])


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.imu1 = IMU()
        self.imu2 = IMU()
        self.ser = serial.Serial("COM3", 38400, timeout=0.1)
        self.iter_x = 0


        self.figure = pg.PlotWidget(title="Pitch, roll, yaw")
        self.setCentralWidget(self.figure)
        self.figure.setRange(yRange=[-180,180])
        self.figure.addLegend()

        self.x = list(range(RESOLUTION))
        # self.y_roll = self.y_pitch = self.y_yaw = [0 for _ in range(RESOLUTION)]

        self.figure.setBackground('w')

        pen = lambda color: pg.mkPen(color=color, width=3)
        self.roll_line = self.figure.plot(self.x, self.imu1.roll, pen=pen((255, 0, 0)), name='roll')
        self.pitch_line = self.figure.plot(self.x, self.imu1.pitch, pen=pen((0, 255, 0)), name='pitch')
        self.yaw_line = self.figure.plot(self.x, self.imu1.yaw, pen=pen((0, 0, 255)), name='yaw')
        # self.filler_line = self.figure.plot(range(0, 100), [0 for _ in range(100)], pen=pen((255, 255, 255)))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def read_data(self):
        # request data by sending a dot
        self.ser.write(b".")  # * encode string to bytes
        line = self.ser.readline()
        angles = line.split(b", ")
        return line

    def get_position(self):
        values = [float(i.strip()) for i in self.read_data().decode('utf-8').split(',') if i]
        #values = self.read_data().decode("utf-8").split(",")
        if len(values) == 6:

            # print("IMU 1")
            # print(f"roll (x): {values[0]}, pitch (y): {values[1]}, yaw (z): {values[2]}")
            # print("IMU 2")
            # print(f"roll (x): {values[3]}, pitch (y): {values[4]}, yaw (z): {values[5]}")
            # print()

            # return values[:3] if imuNum == 1 else values[3:]
            return values # just x for now
        else:
            return [None for _ in range(6)]

    def update_plot_data(self):
        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.iter_x)  # Add a new value 1 higher than the last.
        self.iter_x += 1

        # print(f"***{self.get_position()[:3]}")
        self.imu1.update(*(self.get_position()[:3]))
        self.imu2.update(*(self.get_position()[3:]))

        self.roll_line.setData(self.x, self.imu1.roll)
        self.pitch_line.setData(self.x, self.imu1.pitch)
        self.yaw_line.setData(self.x, self.imu1.yaw)




app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())