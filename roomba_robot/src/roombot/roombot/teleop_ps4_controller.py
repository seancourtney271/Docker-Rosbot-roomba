#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

def expo(x, e):
    # Exponential curve (e ∈ [0,1]): 0 linear, 1 strong curve
    return math.copysign(abs(x) ** (1.0 + e * 2.0), x)

class TankTeleop(Node):
    def __init__(self):
        super().__init__('tank_teleop_node')

        # Parameters (can override via YAML)
        self.declare_parameter('left_axis', 1)     # PS4 left stick vertical
        self.declare_parameter('right_axis', 4)    # PS4 right stick vertical
        self.declare_parameter('deadman_button', 4)  # L1
        self.declare_parameter('estop_button', 1)    # Circle
        self.declare_parameter('scale', 1.0)       # overall scale for duty [-1..1]
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('expo', 0.3)        # 0 linear .. 1 curvy
        self.declare_parameter('slew_rate', 3.0)   # max change per second (unit duty/sec)
        self.declare_parameter('joy_timeout', 0.5) # sec before stop if no /joy

        self.left_axis = self.get_parameter('left_axis').value
        self.right_axis = self.get_parameter('right_axis').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.estop_button = self.get_parameter('estop_button').value
        self.scale = float(self.get_parameter('scale').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.expo_p = float(self.get_parameter('expo').value)
        self.slew_rate = float(self.get_parameter('slew_rate').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)

        self.pub_left = self.create_publisher(Float32, 'cmd_wheel_left', 10)
        self.pub_right = self.create_publisher(Float32, 'cmd_wheel_right', 10)

        self.last_cmd_left = 0.0
        self.last_cmd_right = 0.0
        self.last_joy_time = time.monotonic()
        self.estopped = False

        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.timer = self.create_timer(0.02, self.watchdog)  # 50 Hz

        self.get_logger().info('tank_teleop_node ready (hold L1 to enable, Circle = E-STOP)')

    def joy_cb(self, msg: Joy):
        self.last_joy_time = time.monotonic()

        # E-stop latching
        if self.estop_button < len(msg.buttons) and msg.buttons[self.estop_button] == 1:
            self.estopped = True

        # Deadman must be held to move
        deadman = (self.deadman_button < len(msg.buttons) and msg.buttons[self.deadman_button] == 1)

        # Read axes with safety bounds
        def get_axis(arr, idx):
            return arr[idx] if 0 <= idx < len(arr) else 0.0

        raw_left = -get_axis(msg.axes, self.left_axis)   # invert so up=+1.0
        raw_right = -get_axis(msg.axes, self.right_axis)

        # Deadzone
        def dz(x, z):
            return 0.0 if abs(x) < z else x
        raw_left = dz(raw_left, self.deadzone)
        raw_right = dz(raw_right, self.deadzone)

        # Expo curve + scaling
        cmd_left = self.scale * expo(max(-1.0, min(1.0, raw_left)), self.expo_p)
        cmd_right = self.scale * expo(max(-1.0, min(1.0, raw_right)), self.expo_p)

        # If not enabled or estopped, command 0
        if not deadman or self.estopped:
            cmd_left = 0.0
            cmd_right = 0.0

        # Slew-rate limiting
        dt = 0.02
        max_step = self.slew_rate * dt
        def slew(prev, target):
            if target > prev + max_step:
                return prev + max_step
            if target < prev - max_step:
                return prev - max_step
            return target

        out_left = slew(self.last_cmd_left, cmd_left)
        out_right = slew(self.last_cmd_right, cmd_right)

        self.last_cmd_left = out_left
        self.last_cmd_right = out_right

        self.pub_left.publish(Float32(data=float(out_left)))
        self.pub_right.publish(Float32(data=float(out_right)))

    def watchdog(self):
        # If no joy messages recently, or estopped, publish zeros
        if (time.monotonic() - self.last_joy_time) > self.joy_timeout or self.estopped:
            if self.last_cmd_left != 0.0 or self.last_cmd_right != 0.0:
                self.last_cmd_left = 0.0
                self.last_cmd_right = 0.0
                self.pub_left.publish(Float32(data=0.0))
                self.pub_right.publish(Float32(data=0.0))

def main():
    rclpy.init()
    node = TankTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()