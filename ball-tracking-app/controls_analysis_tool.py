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
            ['y-setpoint', 'y-measurement']
        ]
        titles = [
            'X-Axis', 'Y-Axis'
        ]

        for i, (col1, col2) in enumerate(self.pairs):
            plot = pg.PlotWidget(background='w')
            plot.showGrid(x=True, y=True)
            plot.setTitle(titles[i])
            plot.setLabel('left', 'Value')
            plot.setLabel('bottom', 'Time Step')

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

    def update_data(self, t, plot1, plot2):
        window_len = 100

        if len(self.t) >= window_len:
            self.t.pop(0)
            self.data1.pop(0)
            self.data2.pop(0)
            self.data3.pop(0)
            self.data4.pop(0)

        self.t.append(t)
        self.data1.append(plot1[0])
        self.data2.append(plot1[1])
        self.data3.append(plot2[0])
        self.data4.append(plot2[1])

        for i, _ in enumerate(self.pairs):
            curve1, curve2 = self.curves[i]
            if i == 0:
                y1 = self.data1
                y2 = self.data2
            else:
                y1 = self.data3
                y2 = self.data4
            x = list(range(len(y1)))
            curve1.setData(x, y1)
            curve2.setData(x, y2)


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

        win.update_data(t, [x_set, x_meas], [y_set, y_meas])
        t += 1

    timer.timeout.connect(update)
    timer.start(100)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
