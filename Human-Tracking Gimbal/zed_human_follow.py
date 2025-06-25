import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyzed.sl as sl
import serial
import threading
import time
import math

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 1

class MotorController:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=TIMEOUT)
        self.target_angle = 0.0
        self.current_position = 0.0
        self.running = True
        self.lock = threading.Lock()
        print("Arduino connected. Waiting for data...")

    def send_target_angle(self):
        with self.lock:
            command = f"T{self.target_angle:.4f}\n"
            self.ser.write(command.encode('utf-8'))
            print(f"[SEND] {command.strip()}")

    def read_serial(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("p"):
                    pos_str = line[1:]
                    self.current_position = float(pos_str)
                    print(f"[READ] Position = {self.current_position:.4f} rad")
            except (ValueError, UnicodeDecodeError):
                continue

    def run(self):
        serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        serial_thread.start()

        try:
            while self.running:
                self.send_target_angle()
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

    def set_target_angle(self, angle):
        with self.lock:
            self.target_angle = angle
            print(f"[CMD] New target angle: {self.target_angle:.4f} rad")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Controller stopped.")

class ZEDPersonTracker(Node):
    def __init__(self):
        super().__init__('zed_person_tracker')
        self.motor = MotorController(SERIAL_PORT, BAUD_RATE)
        self.motor.set_target_angle(0)
        threading.Thread(target=self.motor.run, daemon=True).start()

        self.zed = sl.Camera()
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.ULTRA
        init.camera_resolution = sl.RESOLUTION.HD720

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED open error: {err}")
            exit(1)

        pos_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(pos_params)

        body_params = sl.BodyTrackingParameters()
        body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        body_params.enable_tracking = True
        body_params.enable_body_fitting = True
        self.zed.enable_body_tracking(body_params)

        self.timer = self.create_timer(0.1, self.track_human)

    def track_human(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            bodies = self.zed.retrieve_bodies()
            if len(bodies.body_list) > 0:
                target = bodies.body_list[0]  # 追蹤第一個人
                x = target.position[0]
                angle = math.atan2(x, 1.0) 
                self.motor.set_target_angle(angle)
                self.get_logger().info(f"Target person at X={x:.2f}, angle={angle:.2f} rad")

    def destroy_node(self):
        self.motor.stop()
        self.zed.disable_body_tracking()
        self.zed.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDPersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
