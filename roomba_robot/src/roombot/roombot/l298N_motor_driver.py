#!/usr/bin/env python3
import sys
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import pigpio
except Exception as e:
    pigpio = None


class L298NPigpioDriver(Node):
    """
    ROS 2 node to drive an L298N dual H-bridge using pigpio PWM.

    Subscriptions:
      - cmd_wheel_left  (std_msgs/Float32)  range [-1.0, 1.0]
      - cmd_wheel_right (std_msgs/Float32)  range [-1.0, 1.0]

    Parameters (BCM pin numbers):
        # BCM pins (adjust to your wiring)
    -   ENA_MA: 12  # PWM-capable recommended
    -   IN1_MA: 24
    -   IN2_MA: 23        
    -   ENB_MB: 13  # PWM-capable recommended
    -   IN3_MB: 26
    -   IN4_MB: 16      

    Other parameters:
      - pwm_freq_hz (int, default 2000)     # pigpio picks nearest achievable
      - cmd_timeout (float, default 0.5)    # seconds: stop if no commands
      - brake_on_zero (bool, default false) # true=brake (IN1=IN2=HIGH), false=coast (both LOW)
      - invert_left (bool, default false)   # invert left motor direction
      - invert_right (bool, default false)  # invert right motor direction
      - min_duty (float, default 0.0)       # 0..0.4; adds offset to overcome static friction
      - pigpio_host (str, default "")       # blank=local; set to host/IP if pigpiod on host outside container
      - pigpio_port (int, default 8888)
    """

    def __init__(self):
        super().__init__('l298n_driver_node')

        # --- Declare parameters
        self.declare_parameter('ENA_MA', 12)
        self.declare_parameter('IN1_MA', 24)
        self.declare_parameter('IN2_MA', 23)

        self.declare_parameter('ENB_MB', 13)
        self.declare_parameter('IN3_MB', 26)
        self.declare_parameter('IN4_MB', 16)

        self.declare_parameter('pwm_freq_hz', 2000)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('brake_on_zero', False)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('min_duty', 0.0)

        # pigpio daemon connection settings
        self.declare_parameter('pigpio_host', '')
        self.declare_parameter('pigpio_port', 8888)

        if pigpio is None:
            self.get_logger().fatal('pigpio Python module not found. Install with: sudo apt-get install -y python3-pigpio pigpio')
            raise SystemExit(1)

        # --- Read parameters
        pins = {p: int(self.get_parameter(p).value) for p in
                ['ENA_MA', 'IN1_MA', 'IN2_MA', 'ENB_MB', 'IN3_MB', 'IN4_MB']}
        self.driver_side_en = pins['ENA_MA']
        self.driver_side_in1 = pins['IN1_MA']
        self.driver_side_in2 = pins['IN2_MA']
        self.passenger_side_en = pins['ENB_MB']
        self.passenger_side_in3 = pins['IN3_MB']
        self.passenger_side_in4 = pins['IN4_MB']

        self.pwm_freq = int(self.get_parameter('pwm_freq_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.brake_on_zero = bool(self.get_parameter('brake_on_zero').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.min_duty = float(self.get_parameter('min_duty').value)
        self.min_duty = max(0.0, min(0.4, self.min_duty))  # keep reasonable

        host = str(self.get_parameter('pigpio_host').value or '').strip()
        port = int(self.get_parameter('pigpio_port').value)

        # --- Connect to pigpio daemon (local or remote)
        self.pi = pigpio.pi() if host == '' else pigpio.pi(host, port)
        if not self.pi.connected:
            self.get_logger().fatal(
                f'Cannot connect to pigpio daemon (host="{host or "localhost"}", port={port}). '
                'Start it with: sudo pigpiod'
            )
            raise SystemExit(2)

        # --- Configure GPIO
        for pin in [self.driver_side_in1, self.driver_side_in2, self.passenger_side_in3, self.passenger_side_in4, self.driver_side_en, self.passenger_side_en]:
            self.pi.set_mode(pin, pigpio.OUTPUT)

        # Set PWM frequency on enable pins
        # Note: pigpio chooses the nearest achievable frequency; we log the actual.
        actual_left = self.pi.set_PWM_frequency(self.driver_side_en, self.pwm_freq)
        actual_right = self.pi.set_PWM_frequency(self.passenger_side_en, self.pwm_freq)
        # Use default range 0..255; we’ll scale to that.
        self.pi.set_PWM_range(self.driver_side_en, 255)
        self.pi.set_PWM_range(self.passenger_side_en, 255)
        self.pi.set_PWM_dutycycle(self.driver_side_en, 0)
        self.pi.set_PWM_dutycycle(self.passenger_side_en, 0)

        self.get_logger().info(
            f'pigpio connected. PWM freq requested={self.pwm_freq} Hz, actual_left={actual_left} Hz, actual_right={actual_right} Hz'
        )
        self.get_logger().info(
            f'Pins L(in1,in2,en)=({self.driver_side_in1},{self.driver_side_in2},{self.driver_side_en}) '
            f'R(in1,in2,en)=({self.passenger_side_in3},{self.passenger_side_in4},{self.passenger_side_en}) '
            f'brake_on_zero={self.brake_on_zero}, min_duty={self.min_duty}'
        )

        # State
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Float32, 'cmd_wheel_left', self.left_cb, 10)
        self.create_subscription(Float32, 'cmd_wheel_right', self.right_cb, 10)

        # Control loop
        self.timer = self.create_timer(0.02, self.update_outputs)  # 50 Hz

        self.get_logger().info('l298n_driver_node (pigpio) started')

    # ---------- Callbacks ----------

    def left_cb(self, msg: Float32):
        self.left_cmd = self._clamp(msg.data)
        if self.invert_left:
            self.left_cmd = -self.left_cmd
        self.last_cmd_time = self.get_clock().now()

    def right_cb(self, msg: Float32):
        self.right_cmd = self._clamp(msg.data)
        if self.invert_right:
            self.right_cmd = -self.right_cmd
        self.last_cmd_time = self.get_clock().now()

    # ---------- Core control ----------

    @staticmethod
    def _clamp(x):
        try:
            xf = float(x)
        except Exception:
            xf = 0.0
        return max(-1.0, min(1.0, xf))

    def _apply_motor(self, in1, in2, en, cmd):
        """
        cmd in [-1..1]
        Direction via IN pins, speed via PWM duty on EN pin.
        """
        duty = abs(cmd)
        if duty < 1e-6:
            # Stop: choose brake or coast
            self.pi.set_PWM_dutycycle(en, 0)
            if self.brake_on_zero:
                # Brake (both high on L298N)
                self.pi.write(in1, 1)
                self.pi.write(in2, 1)
            else:
                # Coast (both low)
                self.pi.write(in1, 0)
                self.pi.write(in2, 0)
            return

        # Direction
        if cmd > 0.0:
            self.pi.write(in1, 1)
            self.pi.write(in2, 0)
        else:
            self.pi.write(in1, 0)
            self.pi.write(in2, 1)

        # Add a small minimum duty to overcome stiction if requested
        if self.min_duty > 0.0:
            duty = self.min_duty + (1.0 - self.min_duty) * duty

        # Scale to pigpio range 0..255
        dc = int(max(0.0, min(1.0, duty)) * 255.0)
        self.pi.set_PWM_dutycycle(en, dc)

    def update_outputs(self):
        # Timeout safety
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.left_cmd = 0.0
            self.right_cmd = 0.0

        self._apply_motor(self.driver_side_in1, self.driver_side_in2, self.driver_side_en, self.left_cmd)
        self._apply_motor(self.passenger_side_in3, self.passenger_side_in4, self.passenger_side_en, self.right_cmd)

    # ---------- Shutdown ----------

    def destroy_node(self):
        try:
            # Stop PWM and coast
            self.pi.set_PWM_dutycycle(self.driver_side_en, 0)
            self.pi.set_PWM_dutycycle(self.passenger_side_en, 0)
            if self.brake_on_zero:
                self.pi.write(self.driver_side_in1, 1)
                self.pi.write(self.driver_side_in2, 1)
                self.pi.write(self.passenger_side_in3, 1)
                self.pi.write(self.passenger_side_in4, 1)
            else:
                self.pi.write(self.driver_side_in1, 0)
                self.pi.write(self.driver_side_in2, 0)
                self.pi.write(self.passenger_side_in3, 0)
                self.pi.write(self.passenger_side_in4, 0)
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = L298NPigpioDriver()
    # Graceful signal handling
    def handler(sig, frame):
        node.get_logger().info('Shutting down L298N pigpio driver...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    rclpy.spin(node)