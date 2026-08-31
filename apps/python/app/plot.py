import struct

from PyQt5 import QtWidgets
import pyqtgraph as pg

MAX_POINTS = 50
MAX_ROW = 4
WIDTH = 2


class Curve:
    def __init__(self, label, color):
        self.label = label
        self.color = color
        self.data_y = []
        self.ref = None


class Plot:
    def __init__(self, label, curves):
        self.label = label
        self.curves = curves
        self.ref = None


class PlotApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.sample_count = 0
        self.x_data = []
        self.plots = [
            Plot("acceleration",  [Curve("x", "r"), Curve("y", "g"), Curve("z", "b")]),
            Plot("gyroscope",     [Curve("x", "r"), Curve("y", "g"), Curve("z", "b")]),
            Plot("magnetometer",  [Curve("x", "r"), Curve("y", "g"), Curve("z", "b")]),
            Plot("optical flow",  [Curve("dx", "r"), Curve("dy", "g")]),
            Plot("altitude",      [Curve("baro", "r"), Curve("tof", "c")]),
            Plot("power",         [Curve("voltage", "r")]),
            Plot("position",      [Curve("x", "r"), Curve("y", "g"), Curve("z", "b")]),
            Plot("orientation",   [Curve("x", "r"), Curve("y", "g"), Curve("z", "b")]),
        ]

        self.setWindowTitle("STM32 Telemetry Dashboard")
        self.resize(1000, 1000)
        self.win = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.win)
        self.win.setBackground("k")

        for i, plot_obj in enumerate(self.plots):
            row_idx = i % MAX_ROW
            col_idx = i // MAX_ROW

            p = self.win.addPlot(row=row_idx, col=col_idx, title=plot_obj.label.upper())
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis("left").setWidth(60)
            p.getAxis("left").setPen(pg.mkPen("w"))
            p.getAxis("bottom").setPen(pg.mkPen("w"))
            p.addLegend(labelTextSize="9pt").setLabelTextColor("w")

            plot_obj.ref = p
            p.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
            for curve in plot_obj.curves:
                curve.ref = p.plot([], [], pen=pg.mkPen(color=curve.color, width=WIDTH), name=curve.label)

            if i > 0:
                p.setXLink(self.plots[0].ref)

    def receive_callback(self, enc_byte, payload_bytes):
        self.x_data.append(self.sample_count)
        self.sample_count += 1

        if len(self.x_data) > MAX_POINTS * 2:
            self.x_data.pop(0)

        byte_cursor = 0
        for i in range(8):
            bit_active = (enc_byte >> (7 - i)) & 1
            plot_obj = self.plots[i]

            if bit_active:
                num_curves = len(plot_obj.curves)
                bytes_to_read = num_curves * 4
                data_slice = payload_bytes[byte_cursor : byte_cursor + bytes_to_read]
                byte_cursor += bytes_to_read

                if len(data_slice) == bytes_to_read:
                    parsed_floats = struct.unpack(f"<{num_curves}f", data_slice)
                    for curve_idx, float_val in enumerate(parsed_floats):
                        plot_obj.curves[curve_idx].data_y.append(float_val)
                else:
                    for curve_obj in plot_obj.curves:
                        curve_obj.data_y.append(curve_obj.data_y[-1] if curve_obj.data_y else 0.0)
            else:
                for curve_obj in plot_obj.curves:
                    curve_obj.data_y.append(curve_obj.data_y[-1] if curve_obj.data_y else 0.0)

            for curve_obj in plot_obj.curves:
                if len(curve_obj.data_y) > MAX_POINTS * 2:
                    curve_obj.data_y.pop(0)
                curve_obj.ref.setData(self.x_data, curve_obj.data_y)

        self.plots[0].ref.setXRange(max(0, self.sample_count - MAX_POINTS), self.sample_count, padding=0)

    def closeEvent(self, event):
        event.accept()