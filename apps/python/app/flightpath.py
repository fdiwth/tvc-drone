import asyncio
import sys
import time

import pyqtgraph as pg
from PyQt5 import QtWidgets

SEND_INTERVAL = 0.1
COLUMNS = ["time (s)", "x (m)", "y (m)", "z (m)"]


def load_waypoints(filepath):
    waypoints = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            try:
                t, x, y, z = (float(p) for p in parts)
            except ValueError:
                continue
            waypoints.append((t, x, y, z))

    if not waypoints:
        raise ValueError("no waypoints found in file")

    waypoints.sort(key=lambda w: w[0])
    return waypoints


def interpolate(waypoints, t):
    if t <= waypoints[0][0]:
        return waypoints[0][1:]

    for (t0, *v0), (t1, *v1) in zip(waypoints, waypoints[1:]):
        if t0 <= t <= t1:
            frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return tuple(a + (b - a) * frac for a, b in zip(v0, v1))

    return waypoints[-1][1:]


class PathPlayer:
    def __init__(self, send_fn, waypoints):
        self.send_fn = send_fn
        self.waypoints = waypoints

    async def run(self):
        start = time.monotonic()
        t_end = self.waypoints[-1][0]

        try:
            while True:
                t = time.monotonic() - start
                x, y, z = interpolate(self.waypoints, t)
                self.send_fn(f"{x:.2f},{y:.2f},{z:.2f}")

                if t >= t_end:
                    print("path complete — holding last reference")
                    return

                await asyncio.sleep(SEND_INTERVAL)
        except asyncio.CancelledError:
            pass
        finally:
            self.send_fn("LAND")
            self.send_fn("SCRIPT_MODE:OFF")


class PathEditorWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flight Path Editor")
        self.resize(1000, 600)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        self.table = QtWidgets.QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.itemChanged.connect(self.on_item_changed)
        layout.addWidget(self.table, 2)

        right = QtWidgets.QVBoxLayout()
        layout.addLayout(right, 3)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("k")
        right.addWidget(self.plot_widget)

        plot = self.plot_widget.addPlot(title="FLIGHT PATH")
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.addLegend(labelTextSize="9pt").setLabelTextColor("w")
        self.curves = {
            "x": plot.plot([], [], pen=pg.mkPen("r", width=2), name="x"),
            "y": plot.plot([], [], pen=pg.mkPen("g", width=2), name="y"),
            "z": plot.plot([], [], pen=pg.mkPen("b", width=2), name="z"),
        }

        button_row = QtWidgets.QHBoxLayout()
        right.addLayout(button_row)

        add_btn = QtWidgets.QPushButton("Add Waypoint")
        add_btn.clicked.connect(self.add_row)
        button_row.addWidget(add_btn)

        remove_btn = QtWidgets.QPushButton("Remove Selected")
        remove_btn.clicked.connect(self.remove_selected)
        button_row.addWidget(remove_btn)

        save_btn = QtWidgets.QPushButton("Save As...")
        save_btn.clicked.connect(self.save_file)
        button_row.addWidget(save_btn)

        load_btn = QtWidgets.QPushButton("Load...")
        load_btn.clicked.connect(self.load_file)
        button_row.addWidget(load_btn)

        self.add_row()

    def add_row(self, t=0.0, x=0.0, y=0.0, z=0.0):
        self.table.blockSignals(True)
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, val in enumerate((t, x, y, z)):
            self.table.setItem(row, col, QtWidgets.QTableWidgetItem(f"{val:.2f}"))
        self.table.blockSignals(False)
        self.refresh_plot()

    def remove_selected(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()}, reverse=True)
        for row in rows:
            self.table.removeRow(row)
        self.refresh_plot()

    def on_item_changed(self, item):
        self.refresh_plot()

    def read_waypoints(self):
        waypoints = []
        for row in range(self.table.rowCount()):
            try:
                values = [float(self.table.item(row, col).text()) for col in range(len(COLUMNS))]
            except (ValueError, AttributeError):
                continue
            waypoints.append(tuple(values))
        waypoints.sort(key=lambda w: w[0])
        return waypoints

    def refresh_plot(self):
        waypoints = self.read_waypoints()
        if not waypoints:
            return
        t = [w[0] for w in waypoints]
        self.curves["x"].setData(t, [w[1] for w in waypoints])
        self.curves["y"].setData(t, [w[2] for w in waypoints])
        self.curves["z"].setData(t, [w[3] for w in waypoints])

    def save_file(self):
        filepath, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Flight Path", "", "Text Files (*.txt)")
        if not filepath:
            return
        with open(filepath, "w") as f:
            f.write("# t,x,y,z\n")
            for w in self.read_waypoints():
                f.write(",".join(f"{v:.3f}" for v in w) + "\n")

    def load_file(self):
        filepath, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Flight Path", "", "Text Files (*.txt)")
        if not filepath:
            return
        try:
            waypoints = load_waypoints(filepath)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Load Failed", str(e))
            return

        self.table.blockSignals(True)
        self.table.setRowCount(0)
        self.table.blockSignals(False)
        for w in waypoints:
            self.add_row(*w)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = PathEditorWindow()
    win.show()
    sys.exit(app.exec_())