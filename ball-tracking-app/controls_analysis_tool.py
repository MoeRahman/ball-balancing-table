import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QGridLayout, QWidget
from PyQt5.QtCore import QTimer

class ControlsWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Controls Plot")
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        grid_widget = QWidget()
        grid_layout = QGridLayout()
        grid_widget.setLayout(grid_layout)
        main_layout.addWidget(grid_widget)

        self.plot_widgets = []
        self.curves = []
        self.pairs = [
            ['x-setpoint', 'x-measurement'],
            ['y-setpoint', 'y-measurement'],
            ['x-velocity', 'y-velocity'],
            ['roll_out', 'pitch_out']
        ]
        titles = [
            'X-Axis', 'Y-Axis', 
            'Velocity Estimate', 'Control Output'
        ]

        for i, (col1, col2) in enumerate(self.pairs):
            plot = pg.PlotWidget(background='w')
            plot.showGrid(x=True, y=True)
            plot.setTitle(titles[i])
            plot.setLabel('bottom', 'Time(sec)')
            plot.addLegend()

            curve1 = plot.plot(pen=pg.mkPen('r', width=2), name=col1)
            curve2 = plot.plot(pen=pg.mkPen('b', width=2), name=col2)

            self.plot_widgets.append(plot)
            self.curves.append((curve1, curve2))

            row, col = divmod(i, 2)
            grid_layout.addWidget(plot, row, col)

        self.t = []
        self.data1 = []
        self.data2 = []
        self.data3 = []
        self.data4 = []
        self.x_vel = []
        self.y_vel = []
        self.r_out = []
        self.p_out = []

    def update_data(self, t, plot1, plot2, plot3, plot4):
        window_len = 100

        if len(self.t) >= window_len:
            self.t.pop(0)
            self.data1.pop(0)
            self.data2.pop(0)
            self.data3.pop(0)
            self.data4.pop(0)
            self.x_vel.pop(0)
            self.y_vel.pop(0)
            self.r_out.pop(0)
            self.p_out.pop(0)

        self.t.append(t)
        self.data1.append(plot1[0])
        self.data2.append(plot1[1])
        self.data3.append(plot2[0])
        self.data4.append(plot2[1])
        self.x_vel.append(plot3[0])
        self.y_vel.append(plot3[1])
        self.r_out.append(plot4[0])
        self.p_out.append(plot4[1])

        for i, _ in enumerate(self.pairs):
            curve1, curve2 = self.curves[i]

            match i:
                case 0:
                    y1 = self.data1
                    y2 = self.data2
                case 1:
                    y1 = self.data3
                    y2 = self.data4
                case 2:
                    y1 = self.x_vel
                    y2 = self.y_vel
                case 3:
                    y1 = self.r_out
                    y2 = self.p_out

            curve1.setData(self.t, y1)
            curve2.setData(self.t, y2)


def main():
    app = QApplication(sys.argv)
    win = ControlsWindow()
    win.show()

    # Demo data update
    timer = QTimer()
    t = 0

    def update():
        nonlocal t
        import math

        x_set = 100 * math.sin(0.1 * t)
        x_meas = x_set + 10 * math.sin(0.3 * t)
        y_set = 100 * math.cos(0.1 * t)
        y_meas = y_set + 5 * math.sin(0.2 * t)
        x_vel = 10 * math.sin(0.2 * t)
        y_vel = 10 * math.cos(0.2 * t)
        r_out = math.cos(0.25 * t)
        p_out = math.sin(0.25 * t + 3) + 0.5

        win.update_data(t, [x_set, x_meas], [y_set, y_meas], 
                           [x_vel, y_vel],  [r_out, p_out])
        t += 1

    timer.timeout.connect(update)
    timer.start(100)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
