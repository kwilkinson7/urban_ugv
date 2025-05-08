import time
import math

class MotionController:
    def __init__(self, rosmaster):
        self.rosmaster = rosmaster
        self.manual_mode = False

    def enable_manual_control(self):
        self.manual_mode = True
        print("Switched to manual control")

    def disable_manual_control(self):
        self.manual_mode = False
        print("Switched to autonomous control")

    def stop(self):
        self.rosmaster.set_motor(0, 0, 0, 0)

    def set_velocity(self, vx, vy, omega):
        if self.manual_mode:
            print("Manual control enabled. Ignoring autonomous velocity command.")
            return

        # Simple Mecanum kinematic model
        # vx: forward/backward [-1, 1]
        # vy: left/right [-1, 1]
        # omega: rotation [-1, 1]
        max_input = max(abs(vx), abs(vy), abs(omega))
        if max_input > 1:
            vx /= max_input
            vy /= max_input
            omega /= max_input

        # Wheel speed calculation (simple normalized values)
        front_left  = vx - vy - omega
        front_right = vx + vy + omega
        rear_left   = vx + vy - omega
        rear_right  = vx - vy + omega

        # Normalize if needed
        max_val = max(abs(front_left), abs(front_right), abs(rear_left), abs(rear_right))
        if max_val > 1:
            front_left  /= max_val
            front_right /= max_val
            rear_left   /= max_val
            rear_right  /= max_val

        # Scale to PWM range [-100, 100]
        pwm = lambda v: int(v * 100)
        self.rosmaster.set_motor(
            pwm(front_left),
            pwm(front_right),
            pwm(rear_left),
            pwm(rear_right)
        )

    def test_patterns(self):
        print("Forward")
        self.set_velocity(0.5, 0.0, 0.0)
        time.sleep(1)
        self.stop()
        time.sleep(0.5)

        print("Strafe right")
        self.set_velocity(0.0, 0.5, 0.0)
        time.sleep(1)
        self.stop()
        time.sleep(0.5)

        print("Rotate clockwise")
        self.set_velocity(0.0, 0.0, 0.5)
        time.sleep(1)
        self.stop()
        time.sleep(0.5)

        print("Diagonal (forward + right)")
        self.set_velocity(0.5, 0.5, 0.0)
        time.sleep(1)
        self.stop()
        time.sleep(0.5)

        print("Spiral (forward + rotation)")
        self.set_velocity(0.4, 0.0, 0.3)
        time.sleep(2)
        self.stop()
