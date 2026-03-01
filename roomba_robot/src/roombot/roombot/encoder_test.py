#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadrature encoder test on Raspberry Pi using pigpio.
- Works with pigpiod (daemon) already running
- Decodes direction with a 2-bit state machine
- Reports ticks, dir, frequency (Hz), RPM, and linear speed (m/s)
"""

import time
import math
import pigpio
import signal
import sys

# ========= USER CONFIG =========
# BCM GPIO numbers for this wheel's encoder channels
GPIO_A = 17       # e.g., Yellow wire
GPIO_B = 27       # e.g., Green wire

# Electrical counts per motor shaft revolution (per your listing: 11 signals per turn).
# Many low-cost motors output 11 transitions *per channel*. In quadrature, you often count 4x.
# Start with 44 ticks per shaft rev (11 * 4). Adjust if your motor differs.
CPR_MOTOR = 44

# If the encoder is on the motor shaft but your wheel is AFTER a gearbox,
# multiply by gear ratio to get ticks per WHEEL revolution.
GEAR_RATIO = 30.0      # <-- set your gearbox ratio; e.g., 30:1
CPR_WHEEL = CPR_MOTOR * GEAR_RATIO

# Wheel geometry for linear speed estimate
WHEEL_DIAMETER_M = 0.065   # 65 mm wheel; change to your wheel
SAMPLE_PERIOD = 0.25       # seconds between prints

# Debounce / noise filtering (µs): ignore pulses shorter than this
GLITCH_FILTER_US = 100     # 100 µs is a good starting point near DC motors

# Optional: enable internal pull-ups
USE_PULLUPS = True
# ========= END USER CONFIG =========


class QuadDecoder:
    """
    Quadrature decoder using pigpio edge callbacks and a 2-bit state machine.
    """
    # State transition table (previous << 2 | current) → delta
    # Valid transitions (Gray code): +1 or -1; invalid/bounce → 0
    _TRANSITION = {
         0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
         0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1
    }

    def __init__(self, pi: pigpio.pi, gpio_a: int, gpio_b: int):
        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b

        # Running state
        self.ticks = 0
        self._last_state = 0
        self._last_time = time.time()
        self._edge_count = 0

        # Configure pins
        self.pi.set_mode(self.gpio_a, pigpio.INPUT)
        self.pi.set_mode(self.gpio_b, pigpio.INPUT)
        if USE_PULLUPS:
            self.pi.set_pull_up_down(self.gpio_a, pigpio.PUD_UP)
            self.pi.set_pull_up_down(self.gpio_b, pigpio.PUD_UP)

        # Optional: glitch filter to suppress very short spikes
        if GLITCH_FILTER_US and GLITCH_FILTER_US > 0:
            self.pi.set_glitch_filter(self.gpio_a, GLITCH_FILTER_US)
            self.pi.set_glitch_filter(self.gpio_b, GLITCH_FILTER_US)

        # Initialize last state
        a = self.pi.read(self.gpio_a)
        b = self.pi.read(self.gpio_b)
        self._last_state = (a << 1) | b

        # Register callbacks on both edges for both pins
        self._cb_a = self.pi.callback(self.gpio_a, pigpio.EITHER_EDGE, self._edge)
        self._cb_b = self.pi.callback(self.gpio_b, pigpio.EITHER_EDGE, self._edge)

    def _edge(self, gpio, level, tick):
        # Read both channels to construct current state atomically
        a = self.pi.read(self.gpio_a)
        b = self.pi.read(self.gpio_b)
        state = (a << 1) | b

        key = (self._last_state << 2) | state
        delta = self._TRANSITION.get(key, 0)
        if delta:
            self.ticks += delta
            self._edge_count += 1
        self._last_state = state

    def sample_and_reset_edges(self):
        """
        Return edges seen since last call (for frequency estimate) and reset edge counter.
        """
        now = time.time()
        dt = now - self._last_time
        edges = self._edge_count
        self._edge_count = 0
        self._last_time = now
        return edges, dt

    def shutdown(self):
        self._cb_a.cancel()
        self._cb_b.cancel()
        # Clear filters (optional)
        self.pi.set_glitch_filter(self.gpio_a, 0)
        self.pi.set_glitch_filter(self.gpio_b, 0)


def main():
    pi = pigpio.pi()  # connects to localhost pigpiod
    if not pi.connected:
        print("ERROR: Could not connect to pigpio daemon. Is pigpiod running? Try: sudo pigpiod")
        sys.exit(1)

    dec = QuadDecoder(pi, GPIO_A, GPIO_B)

    # Graceful shutdown on Ctrl-C
    def _sigint(_sig, _frm):
        print("\nStopping…")
        dec.shutdown()
        pi.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    print("Running. Press Ctrl-C to stop.\n")
    print("Cols: ticks | dir | edges/s | RPM (wheel) | lin vel (m/s)")

    circumference = math.pi * WHEEL_DIAMETER_M
    while True:
        time.sleep(SAMPLE_PERIOD)
        edges, dt = dec.sample_and_reset_edges()
        edges_per_sec = edges / dt if dt > 0 else 0.0

        # Each valid transition increments 'ticks' by 1 (4x decoding).
        # Convert tick rate to rev/s at the wheel:
        # rev/s = (ticks/s) / CPR_WHEEL
        tick_rate = edges_per_sec  # edges == ticks with 4x table above
        rev_per_sec = tick_rate / CPR_WHEEL if CPR_WHEEL else 0.0
        rpm = rev_per_sec * 60.0

        # Direction from last few edges: compare sign over the last period if any movement
        dir_str = "0"
        # Heuristic: look at tick difference over the window
        # (You could track delta ticks inside the decoder similarly.)
        # For simplicity, re-read absolute ticks and store previous locally:
        # We'll keep a small static attribute on the function:
        if not hasattr(main, "_last_ticks"):
            main._last_ticks = dec.ticks
        delta_ticks = dec.ticks - main._last_ticks
        main._last_ticks = dec.ticks
        if delta_ticks > 0:
            dir_str = "+"
        elif delta_ticks < 0:
            dir_str = "-"

        # Linear velocity estimate
        lin_vel = rev_per_sec * circumference  # m/s

        print(f"{dec.ticks:7d} | {dir_str:3s} | {edges_per_sec:7.1f} | {rpm:11.3f} | {lin_vel:10.4f}")

if __name__ == "__main__":
    main()
