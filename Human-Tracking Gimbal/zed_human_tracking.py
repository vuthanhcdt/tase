import pyzed.sl as sl
import serial
import time
import threading
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
        """Send the target angle to Arduino using a command string."""
        with self.lock:
            command = f"T{self.target_angle:.4f}\n"
            self.ser.write(command.encode('utf-8'))
            print(f"[SEND] {command.strip()}")

    def read_serial(self):
        """Read and parse serial data from Arduino."""
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
        """Main loop for sending target angle periodically."""
        serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        serial_thread.start()

        try:
            while self.running:
                self.send_target_angle()
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

    def set_target_angle(self, angle):
        """Set the new desired angle."""
        with self.lock:
            self.target_angle = angle
            print(f"[CMD] New target angle: {self.target_angle:.4f} rad")

    def stop(self):
        """Stop the control loop and close the serial port."""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Controller stopped.")


def main():
    # Initialize ZED camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED: {status}")
        exit(1)

    # Enable object detection
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    obj_param.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX_FAST

    status = zed.enable_object_detection(obj_param)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to enable object detection: {status}")
        zed.close()
        exit(1)

    # Initialize motor controller and start control thread
    controller = MotorController(SERIAL_PORT, BAUD_RATE)
    threading.Thread(target=controller.run, daemon=True).start()

    # Object detection loop
    runtime_parameters = sl.RuntimeParameters()
    objects = sl.Objects()

    print("Human tracking started... Press Ctrl+C to stop.")

    try:
        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_objects(objects)
                if objects.is_new and len(objects.object_list) > 0:
                    for obj in objects.object_list:
                        if obj.label == sl.OBJECT_CLASS.PERSON and obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                            pos = obj.position
                            x, z = pos[0], pos[2]  # X: left/right, Z: forward/backward
                            yaw = math.atan2(x, z)  # Compute angle to person
                            controller.set_target_angle(yaw)
                            break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Tracking interrupted. Shutting down...")
        controller.stop()
        zed.disable_object_detection()
        zed.close()

if __name__ == "__main__":
    main()
