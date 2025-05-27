#!/usr/bin/env python

import sys
import threading
import time

from typing import Callable

import serial

from serial.tools import list_ports

BAUDRATE = 1_000_000
DEFAULT_WIN32_PORT = "COM3"
PAYLOAD_CHUNK_SIZE = 64
PAYLOAD_CHUNK_DELAY = 0.005  # 5 ms

TEST_STRING = b"""ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
ABCDEFGHIJKLMNOPQRSTUVWXYZ\
"""

MAX_PAYLOAD_SIZE = 256


def get_default_port():
    """Return default serial port."""
    ports = [port for port in list_ports.comports()]
    if sys.platform != "win32":
        ports = [port for port in ports if "J-Link" == port.product]
    else:
        return DEFAULT_WIN32_PORT
    if not ports:
        return "/dev/ttyACM0"
    # return first JLink port available
    return ports[0].device


class SerialInterfaceException(Exception):
    """Exception raised when serial port is disconnected."""


class SerialInterface(threading.Thread):
    """Bidirectional serial interface."""

    def __init__(self, port: str, baudrate: int, callback: Callable):
        self.lock = threading.Lock()
        self.callback = callback
        self.serial = serial.Serial(port, baudrate)
        super().__init__(daemon=True)

    def run(self):
        """Listen continuously at each byte received on serial."""
        self.serial.flush()
        try:
            while 1:
                try:
                    length = int.from_bytes(self.serial.read(1), "big")
                    try:
                        bytes_ = self.serial.read(length)
                        if len(bytes_) != length:
                            print(f"Error: expected {length} bytes, got {len(bytes_)}")
                            continue
                    except (TypeError, serial.serialutil.SerialException) as exc:
                        print(f"Error reading bytes: {exc}")
                        pass
                    else:
                        self.callback(bytes_)
                except (TypeError, serial.serialutil.SerialException):
                    length = None
                if length is None:
                    print("Serial port disconnected")
                    break
        except serial.serialutil.PortNotOpenError as exc:
            raise SerialInterfaceException(f"{exc}") from exc
        except serial.serialutil.SerialException as exc:
            raise SerialInterfaceException(f"{exc}") from exc

    def stop(self):
        self.serial.close()
        self.join()

    def write(self, bytes_):
        """Write bytes on serial."""
        # Send 64 bytes at a time
        self.serial.write(len(bytes_).to_bytes(1, "big"))
        self.serial.flush()
        pos = 0
        while (pos % PAYLOAD_CHUNK_SIZE) == 0 and pos < len(bytes_):
            self.serial.write(bytes_[pos : pos + PAYLOAD_CHUNK_SIZE])
            self.serial.flush()
            pos += PAYLOAD_CHUNK_SIZE
        self.serial.flush()


class Sender:
    """Sender class to send data over serial interface."""

    def __init__(self):
        self.serial_interface = SerialInterface(get_default_port(), BAUDRATE, self.callback)
        self.serial_interface.start()
        self.received = None

    def callback(self, bytes_):
        print(f"Received: {bytes_} ({len(bytes_)}B)")
        self.received = bytes_

    def send(self, data: bytes):
        """Send data over serial."""
        self.serial_interface.serial.flush()
        self.sent = data
        self.serial_interface.write(data)

    def stop(self):
        """Stop the serial interface."""
        self.serial_interface.stop()


def main():
    sender = Sender()
    try:
        size = MAX_PAYLOAD_SIZE - 1
        while size >= 0:
            data = TEST_STRING[:size]
            sender.send(data)
            size -= 1
            time.sleep(0.008)
            if data:
                assert sender.received is not None, "No data received"
                assert sender.received == data, f"{sender.received} != {data}"
            sender.received = None

    except KeyboardInterrupt:
        sender.stop()
        print("Serial interface stopped.")


if __name__ == "__main__":
    main()
