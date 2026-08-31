import asyncio
import sys

from flightpath import load_waypoints, PathPlayer

LAUNCH_SETTLE_DELAY = 0.5


class CommandApp:
    def __init__(self):
        self.protocol = None
        self.path_task = None

    def set_protocol(self, protocol):
        self.protocol = protocol

    def receive_callback(self, enc_byte, payload_bytes):
        pass

    def _send(self, cmd):
        packet = cmd.encode('utf-8') + b'\n'
        self.protocol.transport.write(packet)
        print(f"sent: {cmd}")

    async def terminal_loop(self):
        loop = asyncio.get_running_loop()

        while True:
            user_input = await loop.run_in_executor(None, sys.stdin.readline)
            cmd = user_input.strip()

            if not cmd:
                continue

            if not (self.protocol and self.protocol.transport):
                print("serial connection not established yet.")
                continue

            if not self.protocol.handshake_complete:
                print("warning: Handshake not yet completed. command may be ignored by drone.")

            if self.path_task and not self.path_task.done():
                self.path_task.cancel()
                print("path playback interrupted")

            if cmd.upper().startswith("LAUNCH:"):
                filepath = cmd[len("LAUNCH:"):].strip()
                try:
                    waypoints = load_waypoints(filepath)
                except Exception as e:
                    print(f"failed to load path file: {e}")
                    continue

                self._send("SCRIPT_MODE:ON")
                self._send("LAUNCH")
                await asyncio.sleep(LAUNCH_SETTLE_DELAY)
                player = PathPlayer(self._send, waypoints)
                self.path_task = asyncio.create_task(player.run())
            else:
                self._send(cmd)