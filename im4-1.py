from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import serial

RESOLUTION = 100


class IMU:
    def __init__(self):
        self.pitch = [0 for _ in range(RESOLUTION)]
        self.roll = [0 for _ in range(RESOLUTION)]
        self.yaw = [0 for _ in range(RESOLUTION)]
        self.x = [0 for _ in range(RESOLUTION)]
        self._axes = [self.pitch, self.roll, self.yaw]

        self.roll_line = None
        self.pitch_line = None
        self.yaw_line = None
        self._lines = [self.roll_line, self.pitch_line, self.yaw]

        self.iter_x = 0

    def plot(self, figure):
        pen = lambda color: pg.mkPen(color=color, width=3)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        names = ["roll", "pitch", "yaw"]
        for i, axis in enumerate(self._axes):
            self._lines[i] = figure.plot(
                self.x, axis, pen=pen(colors[i]), name=names[i]
            )

    def update(self, x: float, y: float, z: float):
        for axis, line, point in zip(self._axes, self._lines, [x, y, z]):
            axis.pop(0)
            axis.append(point if point is not None else axis[-1])
            line.setData(self.x, axis)

        self.x.pop(0)
        self.x.append(self.iter_x)
        self.iter_x += 1


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.imu1 = IMU()
        self.imu2 = IMU()
        self.ser = serial.Serial("COM3", 38400, timeout=0.1)

        self.figure = pg.PlotWidget(title="Pitch, roll, yaw")
        self.setCentralWidget(self.figure)
        self.figure.setRange(yRange=[-180, 180])
        self.figure.addLegend()

        self.x = list(range(RESOLUTION))

        self.imu1.plot(figure=self.figure)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def read_data(self):
        # request data by sending a dot
        self.ser.write(b".")  # * encode string to bytes
        line = self.ser.readline()
        return line

    def get_position(self):
        values = [
            float(i.strip()) for i in self.read_data().decode("utf-8").split(",") if i
        ]
        if len(values) == 6:

            # print("IMU 1")
            # print(f"roll (x): {values[0]}, pitch (y): {values[1]}, yaw (z): {values[2]}")
            # print("IMU 2")
            # print(f"roll (x): {values[3]}, pitch (y): {values[4]}, yaw (z): {values[5]}")
            # print()

            return values
        return [None for _ in range(6)]

    def update_plot_data(self):
        self.imu1.update(*(self.get_position()[:3]))
        # self.imu2.update(*(self.get_position()[3:]))


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())
