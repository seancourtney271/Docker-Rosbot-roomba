#!/usr/bin/env python3
# ROS 2 Humble encoder-based odometry node using pigpio (Pi 4B)
# Requires: pigpiod running, python3-pigpio installed
import math, time, signal, sys
import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

# ======= USER PARAM DEFAULTS (override via ROS params) =======
# GPIO: Right/passenger (A=17, B=27), Left/driver (A=5, B=6)
DEFAULT_GPIO_RIGHT_A = 17
DEFAULT_GPIO_RIGHT_B = 27
DEFAULT_GPIO_LEFT_A  = 5
DEFAULT_GPIO_LEFT_B  = 6

# Encoder electrical CPR at motor shaft (signals per rev * 4 if using 4x decoding)
# Your product image says "11 signals/turn per channel"; with 4x decoding, start with 44 CPR motor.
DEFAULT_CPR_MOTOR = 44
DEFAULT_GEAR_RATIO = 15.5454  
DEFAULT_CPR_WHEEL = DEFAULT_CPR_MOTOR * DEFAULT_GEAR_RATIO  # ticks per wheel revolution (4x)

# Geometry
DEFAULT_WHEEL_DIAMETER = 0.0662   # 66.2 mm wheel
DEFAULT_TRACK_WIDTH    = 0.23495 # 9.25 Inches    # distance between wheel contact patches (m) - measure yours!

# Filters
DEFAULT_GLITCH_US = 100          # 100 µs to tame motor noise; reduce if missing pulses

# Frame IDs
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_LINK  = "base_link"

class QuadDecoder:
    _TRANSITION = {
        0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
        0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1
    }
    def __init__(self, pi: pigpio.pi, a: int, b: int, glitch_us: int = 100, pullups=True):
        self.pi = pi; self.a = a; self.b = b
        self.ticks = 0
        self._last_state = 0
        self._edge_count = 0
        self.pi.set_mode(a, pigpio.INPUT); self.pi.set_mode(b, pigpio.INPUT)
        if pullups:
            self.pi.set_pull_up_down(a, pigpio.PUD_UP)
            self.pi.set_pull_up_down(b, pigpio.PUD_UP)
        if glitch_us > 0:
            self.pi.set_glitch_filter(a, glitch_us)
            self.pi.set_glitch_filter(b, glitch_us)
        ai = self.pi.read(a); bi = self.pi.read(b)
        self._last_state = (ai << 1) | bi
        self._cb_a = self.pi.callback(a, pigpio.EITHER_EDGE, self._edge)
        self._cb_b = self.pi.callback(b, pigpio.EITHER_EDGE, self._edge)

    def _edge(self, gpio, level, tick):
        a = self.pi.read(self.a); b = self.pi.read(self.b)
        state = (a << 1) | b
        key = (self._last_state << 2) | state
        delta = self._TRANSITION.get(key, 0)
        if delta:
            self.ticks += delta
            self._edge_count += 1
        self._last_state = state

    def get_and_reset_edges(self):
        e = self._edge_count
        self._edge_count = 0
        return e

    def shutdown(self):
        self._cb_a.cancel(); self._cb_b.cancel()
        self.pi.set_glitch_filter(self.a, 0); self.pi.set_glitch_filter(self.b, 0)

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("encoder_odom_node")

        # ---- parameters
        self.declare_parameter("gpio_right_a", DEFAULT_GPIO_RIGHT_A)
        self.declare_parameter("gpio_right_b", DEFAULT_GPIO_RIGHT_B)
        self.declare_parameter("gpio_left_a",  DEFAULT_GPIO_LEFT_A)
        self.declare_parameter("gpio_left_b",  DEFAULT_GPIO_LEFT_B)
        self.declare_parameter("cpr_wheel",    DEFAULT_CPR_WHEEL)
        self.declare_parameter("wheel_diameter", DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter("track_width",    DEFAULT_TRACK_WIDTH)
        self.declare_parameter("glitch_filter_us", DEFAULT_GLITCH_US)
        self.declare_parameter("odom_frame", DEFAULT_ODOM_FRAME)
        self.declare_parameter("base_frame", DEFAULT_BASE_LINK)
        self.declare_parameter("publish_debug_ticks", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("rate_hz", 50.0)  # odom update rate

        GA = self.get_parameter("gpio_right_a").value
        GB = self.get_parameter("gpio_right_b").value
        LA = self.get_parameter("gpio_left_a").value
        LB = self.get_parameter("gpio_left_b").value
        glitch = int(self.get_parameter("glitch_filter_us").value)
        self.cpr = float(self.get_parameter("cpr_wheel").value)
        self.wheel_diam = float(self.get_parameter("wheel_diameter").value)
        self.track = float(self.get_parameter("track_width").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        # ---- pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal("Cannot connect to pigpiod. Start it with: sudo pigpiod")
            sys.exit(1)
        self.q_right = QuadDecoder(self.pi, GA, GB, glitch)
        self.q_left  = QuadDecoder(self.pi, LA, LB, glitch)

        # ---- state
        self.last_ticks_r = 0
        self.last_ticks_l = 0
        self.last_time = self.get_clock().now()
        self.x = 0.0; self.y = 0.0; self.th = 0.0

        # ---- pubs
        if bool(self.get_parameter("publish_debug_ticks").value):
            self.pub_tr = self.create_publisher(Int32, "wheel_ticks_right", 10)
            self.pub_tl = self.create_publisher(Int32, "wheel_ticks_left", 10)
        else:
            self.pub_tr = None; self.pub_tl = None
        self.pub_odom = self.create_publisher(Odometry, "odom", 50)

        # ---- TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- timer
        self.timer = self.create_timer(1.0/self.rate_hz, self._update)

        self.get_logger().info(f"Encoder odom started: R({GA},{GB}) L({LA},{LB}) cpr={self.cpr}, "
                               f"wheel_d={self.wheel_diam} track={self.track}")

    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        ticks_r = self.q_right.ticks
        ticks_l = self.q_left.ticks
        dticks_r = ticks_r - self.last_ticks_r
        dticks_l = ticks_l - self.last_ticks_l
        self.last_ticks_r = ticks_r
        self.last_ticks_l = ticks_l
        self.last_time = now

        # ticks -> meters
        # wheel revs = ticks / cpr ; distance = revs * (pi * D)
        circ = math.pi * self.wheel_diam
        dR = (dticks_r / self.cpr) * circ
        dL = (dticks_l / self.cpr) * circ

        dS = 0.5 * (dR + dL)
        dTh = (dR - dL) / self.track

        # integrate pose (2nd-order midpoint)
        if abs(dTh) < 1e-9:
            self.x += dS * math.cos(self.th)
            self.y += dS * math.sin(self.th)
        else:
            self.x += dS * math.cos(self.th + dTh/2.0)
            self.y += dS * math.sin(self.th + dTh/2.0)
        self.th = self._normalize(self.th + dTh)

        # velocities
        v = dS / dt
        w = dTh / dt

        # ---- publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.th/2.0); qw = math.cos(self.th/2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Pose covariance (x, y, yaw modest; others very large)
        odom.pose.covariance = [
        0.05,   0.0,    0.0,    0.0,    0.0,    0.0,
        0.0,    0.05,   0.0,    0.0,    0.0,    0.0,
        0.0,    0.0,    1e6,    0.0,    0.0,    0.0,
        0.0,    0.0,    0.0,    1e6,    0.0,    0.0,
        0.0,    0.0,    0.0,    0.0,    1e6,    0.0,
        0.0,    0.0,    0.0,    0.0,    0.0,    0.1
        ]

        # twist in child_frame (base_link)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w
        # Twist covariance (vx, vy, yaw_rate modest; others very large)
        odom.twist.covariance = [
        0.1,    0.0,    0.0,    0.0,    0.0,    0.0,
        0.0,    0.1,    0.0,    0.0,    0.0,    0.0,
        0.0,    0.0,    1e6,    0.0,    0.0,    0.0,
        0.0,    0.0,    0.0,    1e6,    0.0,    0.0,
        0.0,    0.0,    0.0,    0.0,    1e6,    0.0,
        0.0,    0.0,    0.0,    0.0,    0.0,    0.2
        ]

        self.pub_odom.publish(odom)

        # ---- publish TF (odom -> base_link)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # ---- debug ticks
        if self.pub_tr:
            m = Int32(); m.data = ticks_r; self.pub_tr.publish(m)
            m = Int32(); m.data = ticks_l; self.pub_tl.publish(m)

    @staticmethod
    def _normalize(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def destroy_node(self):
        try:
            self.q_right.shutdown()
            self.q_left.shutdown()
            self.pi.stop()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()