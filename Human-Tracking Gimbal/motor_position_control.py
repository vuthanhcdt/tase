import motor_connection as NRSLmotor
import time
from math import pi

# PD gains
kp = 1.00
kd = 0.01

# Loop timing
change_interval = 0.01
last_change_time = time.time()

if __name__ == "__main__":
    motor = NRSLmotor.MotorConnection(port='COM8')
    motor.connect()

    try:
        # Get target position in radians
        target_deg = float(input("Enter target position (deg): "))
        target_rad = target_deg * (pi / 180)
        print(f"Target: {target_deg:.2f}° ({target_rad:.2f} rad)")

        while True:
            data = motor.read_data()
            now = time.time()

            if now - last_change_time >= change_interval and data:
                pos_rad, vel_rad_s = data

                error = target_rad - pos_rad
                cmd_speed = kp * error - kd * vel_rad_s

                pos_deg = pos_rad * (180 / pi)
                print(f"Pos: {pos_deg:.2f}° | Err: {error:.2f} rad | "
                      f"Vel: {vel_rad_s:.2f} rad/s | Cmd: {cmd_speed:.2f}")

                motor.send_speed(cmd_speed)
                last_change_time = now

    except KeyboardInterrupt:
        motor.send_speed(0.0)
        print("\nInterrupted by user. Motor stopped.")

    finally:
        motor.disconnect()
