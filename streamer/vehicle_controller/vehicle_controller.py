import serial
import time
from typing import Optional


class VehicleController:

    def __init__(self, port: str, baud: int, timeout_sec: Optional[float] = 0.1, delay_cmd_ms: Optional[int] = 20):

        self.port = port
        self.baud = baud
        self.timeout_sec = timeout_sec
        self.delay_cmd_ms = delay_cmd_ms

        self.serialCom = None

    def connect(self):
        """
        Connects to the car.
        Is called automatically by __enter__() if the VehicleController is called within a Python "with" statement.
        """
        self.serialCom = serial.Serial(self.port, self.baud, timeout=self.timeout_sec)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()

    def close(self):
        """Disconnects from the car."""
        self.serialCom.close()

    def __enter__(self):
        """This allows the VehicleController to be (optionally) used in Python 'with' statements"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the VehicleController to be (optionally) used in Python 'with' statements"""
        self.close()

    def _send_command(self, command: str):
        command = command.encode('ascii')
        self.serialCom.write(command)
        time.sleep(self.delay_cmd_ms / 1000)

    def send_accel(self, accel: float):

        if accel < 0:
            accel = 0
        elif accel > 1023:
            accel = 1023

        self._send_command(f"ACCEL {int(accel)};\r\n")

    def send_angle(self, angle: float):

        if angle < -33.0:
            angle = -33.0
        elif angle > 33.0:
            angle = 33.0

        self._send_command(f"ANGLE {float(angle)};\r\n")

    def send_brake(self, brake: float):

        if brake < 0:
            brake = 0
        elif brake > 1:
            brake = 1

        self._send_command(f"BRAKE {float(brake)};\r\n")

    def send_combined(self, accel: float, brake: float, angle: float):
        self.send_brake(brake)
        self.send_angle(angle)
        self.send_accel(accel)
