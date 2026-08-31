import asyncio
import sys

import serial_asyncio
from PyQt5 import QtWidgets
from qasync import QEventLoop

import com
from plot import PlotApp
from cal import CalibrationApp
from command import CommandApp
from flightpath import PathEditorWindow

# ── App selection ──────────────────────────────────────────────────────────────
# Add as many apps as you want to this list.
# Available modes: "plot", "cal", "command", "patheditor"
MODES = ["command", "plot"]
# MODES = ["patheditor"]


TARGET_PORT = "/dev/cu.SLAB_USBtoUART"
# TARGET_PORT = "/dev/cu.usbmodem345D397032351"
# ──────────────────────────────────────────────────────────────────────────────

class AppMultiplexer:
    """
    Acts as a middleman between com.py and multiple apps.
    Broadcasts incoming serial data to all running apps and passes
    the active connection protocol to apps that need to send data.
    """
    def __init__(self, apps):
        self.apps = apps

    def set_protocol(self, protocol):
        for app in self.apps:
            if hasattr(app, "set_protocol"):
                app.set_protocol(protocol)

    def receive_callback(self, enc_byte, payload_bytes):
        for app in self.apps:
            if hasattr(app, "receive_callback"):
                app.receive_callback(enc_byte, payload_bytes)


async def run(port_name, multiplexer):
    loop = asyncio.get_running_loop()
    try:
        await serial_asyncio.create_serial_connection(
            loop, lambda: com.SerialProtocol(multiplexer), port_name, baudrate=115200
        )
        while True:
            await asyncio.sleep(3600)
    except Exception as e:
        print(f"Error opening port {port_name}: {e}")


if __name__ == "__main__":
    com.list_ports()

    # 1. Setup the Event Loop
    # Any Qt-backed app ("plot", "patheditor") requires Qt's event loop (QEventLoop)
    needs_qt = "plot" in MODES or "patheditor" in MODES
    if needs_qt:
        qt_app = QtWidgets.QApplication(sys.argv)
        loop = QEventLoop(qt_app)
        asyncio.set_event_loop(loop)
    else:
        qt_app = None
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

    active_apps = []

    # 2. Initialize requested apps
    if "plot" in MODES:
        plot_app = PlotApp()
        plot_app.show()
        active_apps.append(plot_app)

    if "cal" in MODES:
        cal_app = CalibrationApp()
        active_apps.append(cal_app)

    if "command" in MODES:
        cmd_app = CommandApp()
        active_apps.append(cmd_app)
        # The terminal requires its own asynchronous task to constantly listen to the keyboard
        loop.create_task(cmd_app.terminal_loop())

    if "patheditor" in MODES:
        path_editor = PathEditorWindow()
        path_editor.show()
        active_apps.append(path_editor)

    # 3. Create the multiplexer and start the serial connection
    # patheditor is offline-only and shouldn't trigger a connection attempt on its own
    needs_serial = any(mode != "patheditor" for mode in MODES)
    if needs_serial:
        multiplexer = AppMultiplexer(active_apps)
        loop.create_task(run(TARGET_PORT, multiplexer))

    # 4. Run the loop indefinitely
    try:
        if qt_app:
            with loop:
                sys.exit(loop.run_forever())
        else:
            loop.run_forever()
    except KeyboardInterrupt:
        print("\nExiting Ground Station...")
        sys.exit(0)