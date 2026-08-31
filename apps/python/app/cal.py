# CalibrationApp: no Qt window. Pipes IMU data to MotionCal via a socat virtual serial port.
# Requires socat: brew install socat

import os
import struct
import subprocess
import tempfile
import time

# Regular MotionCal doens't detect virtual ports: compiled MotioinCal with modified portlist.cpp

SOCAT_PORT_A = "/tmp/motioncal_in"   # write end  — this app writes here
SOCAT_PORT_B = "/tmp/motioncal_out"  # read end   — point MotionCal at this
SAMPLE_INTERVAL = 5


class CalibrationApp:
    def __init__(self):
        # Remove stale symlinks from a previous run
        for p in (SOCAT_PORT_A, SOCAT_PORT_B):
            try:
                os.remove(p)
            except FileNotFoundError:
                pass

        # socat creates two linked pseudo-ttys exposed as named symlinks
        self._socat = subprocess.Popen(
            [
                "socat",
                f"pty,raw,echo=0,link={SOCAT_PORT_A}",
                f"pty,raw,echo=0,link={SOCAT_PORT_B}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Give socat a moment to create the symlinks
        for _ in range(20):
            if os.path.exists(SOCAT_PORT_A):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("socat did not create virtual ports in time — is socat installed? (brew install socat)")

        self._fd = os.open(SOCAT_PORT_A, os.O_WRONLY | os.O_NOCTTY)
        print(f"virtual serial port ready — point MotionCal at: {SOCAT_PORT_B}")
        self.count = 0

    def receive_callback(self, enc_byte, payload_bytes):
        if self.count >= SAMPLE_INTERVAL:
            float_format = "<fff"
            byte_cursor = 0

            if enc_byte & 0b10000000:
                accel_x, accel_y, accel_z = struct.unpack(float_format, payload_bytes[byte_cursor : byte_cursor + 12])
                byte_cursor += 12
            else:
                accel_x, accel_y, accel_z = 0.0, 0.0, 0.0

            if enc_byte & 0b01000000:
                gyro_x, gyro_y, gyro_z = struct.unpack(float_format, payload_bytes[byte_cursor : byte_cursor + 12])
                byte_cursor += 12
            else:
                gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0

            if enc_byte & 0b00100000:
                mag_x, mag_y, mag_z = struct.unpack(float_format, payload_bytes[byte_cursor : byte_cursor + 12])
            else:
                mag_x, mag_y, mag_z = 0.0, 0.0, 0.0

            # accel as raw counts (±8g → 4096 LSB/g), gyro in dps, mag in uT * 10
            accel_range = 4096
            result = f"Raw:{int(accel_x * accel_range)},{int(accel_y * accel_range)},{int(accel_z * accel_range)},{int(gyro_x)},{int(gyro_y)},{int(gyro_z)},{int(mag_x * 10)},{int(mag_y * 10)},{int(mag_z * 10)}\r\n"
            data = result.encode("utf-8")
            while data:
                n = os.write(self._fd, data)
                data = data[n:]
        self.count += 1
    def close(self):
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._socat.terminate()
