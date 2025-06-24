import serial
import serial.tools.list_ports
import time
import sys
import struct

class MotorConnection:
    """Serial interface to an Arduino-based motor controller."""

    def __init__(self, port=None, baudrate=115200, timeout=1):
        """Set port, baudrate, and timeout."""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

    def auto_detect_port(self):
        """Find the Arduino port automatically."""
        ports = list(serial.tools.list_ports.comports())
        arduino_ports = [p.device for p in ports if 'Arduino' in p.description]

        if not arduino_ports:
            print("No Arduino found.")
            sys.exit(1)
        if len(arduino_ports) == 1:
            print(f"Arduino at {arduino_ports[0]}")
            return arduino_ports[0]

        print("Multiple Arduinos found:")
        for idx, p in enumerate(arduino_ports, 1):
            print(f"{idx}: {p}")
        choice = input("Select port number: ")
        try:
            return arduino_ports[int(choice) - 1]
        except (IndexError, ValueError):
            print("Invalid selection.")
            sys.exit(1)

    def connect(self):
        """Open serial connection."""
        if not self.port:
            self.port = self.auto_detect_port()

        try:
            print(f"Connecting to {self.port} @ {self.baudrate} baud...")
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(3)  # allow Arduino reset
            if self.serial_connection.is_open:
                print("Connection successful.")
            else:
                print("Failed to open serial port.")
                sys.exit(1)
        except serial.SerialException as e:
            print(f"Connection error: {e}")
            sys.exit(1)

    def disconnect(self):
        """Close serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.")
        else:
            print("No open connection to close.")

    def read_data(self):
        """
        Read one line: 'A'+position(float)+speed(float)+'\n'.
        Returns (position, speed) or None if malformed.
        """
        if not (self.serial_connection and self.serial_connection.is_open):
            print("Serial port not open.")
            sys.exit(1)

        try:
            data = self.serial_connection.readline()
            if len(data) == 10 and data[0] == ord('A') and data[-1] == ord('\n'):
                pos, spd = struct.unpack('ff', data[1:9])
                return (round(pos, 2), round(spd, 2))
            return None
        except serial.SerialException as e:
            print(f"Read error: {e}")
            self.disconnect()
            sys.exit(1)
        except struct.error as e:
            print(f"Unpack error: {e}")
            return None

    def send_speed(self, speed):
        """
        Send target speed: 'T'+speed(float)+'\n'.
        Speed is clamped to [-20.0, 20.0].
        """
        if not (self.serial_connection and self.serial_connection.is_open):
            print("Serial port not open.")
            sys.exit(1)

        speed = max(-20.0, min(speed, 20.0))
        msg = b'T' + struct.pack('f', speed) + b'\n'
        try:
            self.serial_connection.write(msg)
        except serial.SerialException as e:
            print(f"Write error: {e}")
            self.disconnect()
            sys.exit(1)
