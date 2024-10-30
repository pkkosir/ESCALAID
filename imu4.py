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

        self.x = list(range(self.resolution))  # 100 time points
        self.y = [0 for _ in range(self.resolution)]  # 100 data points

        self.figure.setBackground('w')

        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line = self.figure.plot(self.x, self.y, pen=pen)

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
            return values[0]  # just x for now
        else:
            return 0

    def update_plot_data(self):

        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.iter_x)  # Add a new value 1 higher than the last.
        self.iter_x += 1

        roll = self.get_position()

        self.y = self.y[1:]  # Remove the first
        self.y.append( roll )  # Add a new random value.
        self.data_line.setData(self.x, self.y)  # Update the data.



app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())