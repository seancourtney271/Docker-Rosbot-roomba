#!/usr/bin/env python3
import signal
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# If running in Docker, ensure /dev/gpiomem is mapped and RPi.GPIO is installed in the image
try:
    import RPi.GPIO as GPIO
except Exception as e:
    GPIO = None

class L298NDriver(Node):
    def __init__(self):
        super().__init__('l298n_driver_node')

        # Parameters: BCM pin numbers
        self.declare_parameter('left_in1', 5)
        self.declare_parameter('left_in2', 6)
        self.declare_parameter('left_en', 12)   # PWM-capable recommended

        self.declare_parameter('right_in1', 13)
        self.declare_parameter('right_in2', 16)
        self.declare_parameter('right_en', 18)  # PWM-capable recommended

        self.declare_parameter('pwm_freq_hz', 1000)  # 1 kHz is fine for L298N
        self.declare_parameter('cmd_timeout', 0.5)

        pins = {p: self.get_parameter(p).value for p in
                ['left_in1','left_in2','left_en','right_in1','right_in2','right_en']}
        self.left_in1 = int(pins['left_in1'])
        self.left_in2 = int(pins['left_in2'])
        self.left_en = int(pins['left_en'])
        self.right_in1 = int(pins['right_in1'])
        self.right_in2 = int(pins['right_in2'])
        self.right_en = int(pins['right_en'])
        self.pwm_freq = int(self.get_parameter('pwm_freq_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        if GPIO is None:
            self.get_logger().fatal('RPi.GPIO not available. Install it and map /dev/gpiomem into the container.')
            raise SystemExit(1)

        GPIO.setmode(GPIO.BCM)
        for pin in [self.left_in1, self.left_in2, self.right_in1, self.right_in2, self.left_en, self.right_en]:
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_left = GPIO.PWM(self.left_en, self.pwm_freq)
        self.pwm_right = GPIO.PWM(self.right_en, self.pwm_freq)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.last_cmd_time = self.get_clock().now()
        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.create_subscription(Float32, 'cmd_wheel_left', self.left_cb, 10)
        self.create_subscription(Float32, 'cmd_wheel_right', self.right_cb, 10)
        self.create_timer(0.02, self.update_outputs)  # 50 Hz

        self.get_logger().info('l298n_driver_node started')

    def left_cb(self, msg):
        self.left_cmd = max(-1.0, min(1.0, float(msg.data)))
        self.last_cmd_time = self.get_clock().now()

    def right_cb(self, msg):
        self.right_cmd = max(-1.0, min(1.0, float(msg.data)))
        self.last_cmd_time = self.get_clock().now()

    def set_motor(self, in1, in2, pwm, duty):
        # duty: -1..1  (negative = reverse)
        if duty >= 0.0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            pwm.ChangeDutyCycle(min(100.0, duty * 100.0))
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            pwm.ChangeDutyCycle(min(100.0, -duty * 100.0))

    def update_outputs(self):
        # Stop if timeout
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.left_cmd = 0.0
            self.right_cmd = 0.0

        self.set_motor(self.left_in1, self.left_in2, self.pwm_left, self.left_cmd)
        self.set_motor(self.right_in1, self.right_in2, self.pwm_right, self.right_cmd)

    def destroy_node(self):
        try:
            self.set_motor(self.left_in1, self.left_in2, self.pwm_left, 0.0)
            self.set_motor(self.right_in1, self.right_in2, self.pwm_right, 0.0)
            self.pwm_left.stop()
            self.pwm_right.stop()
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = L298NDriver()
    def handler(sig, frame):
        node.get_logger().info('Shutting down L298N driver...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    rclpy.spin(node)