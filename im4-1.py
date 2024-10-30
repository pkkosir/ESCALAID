from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint
import serial

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.ser = serial.Serial("COM3", 38400, timeout=0.1)
        self.iter_x = 0
        self.resolution = 100

        self.figure = pg.PlotWidget()
        self.setCentralWidget(self.figure)

        self.x = list(range(self.resolution))
        self.y_roll = self.y_pitch = self.y_yaw = [0 for _ in range(self.resolution)]

        self.figure.setBackground('w')

        pen = lambda color: pg.mkPen(color=color, width=2)
        self.roll_line = self.figure.plot(self.x, self.y_roll, pen=pen((255, 0, 0)))
        self.pitch_line = self.figure.plot(self.x, self.y_pitch, pen=pen((0, 255, 0)))
        self.yaw_line = self.figure.plot(self.x, self.y_yaw, pen=pen((0, 0, 255)))

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

            print("IMU 1")
            print(f"roll (x): {values[0]}, pitch (y): {values[1]}, yaw (z): {values[2]}")
            print("IMU 2")
            print(f"roll (x): {values[3]}, pitch (y): {values[4]}, yaw (z): {values[5]}")
            print()

            # return values[:3] if imuNum == 1 else values[3:]
            return values[0:3]  # just x for now
        else:
            return 0

    def update_plot_data(self):
        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.iter_x)  # Add a new value 1 higher than the last.
        self.iter_x += 1

        roll, pitch, yaw = self.get_position()

        self.y_roll = self.y_roll[1:]  # Remove the first
        self.y_roll.append( roll )  # Add a new random value.
        self.roll_line.setData(self.x, self.y_roll)  # Update the data.

        self.y_pitch = self.y_pitch[1:]  # Remove the first
        self.y_pitch.append( pitch )  # Add a new random value.
        self.pitch_line.setData(self.x, self.y_pitch)  # Update the data.

        self.y_yaw = self.y_yaw[1:]  # Remove the first
        self.y_yaw.append( yaw )  # Add a new random value.
        self.yaw_line.setData(self.x, self.y_yaw)  # Update the data.



app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())