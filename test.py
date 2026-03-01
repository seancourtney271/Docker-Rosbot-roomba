#!/usr/bin/env python3
"""
Quick BNO055 sanity test over I2C on Raspberry Pi.

- Assumes I2C bus 1 (Raspberry Pi)
- Default address 0x28 (ADR low). Use --addr 0x29 if ADR is high.
- Checks chip ID, configures NDOF mode, prints Euler angles and calibration.

Run:
  python3 bno055_i2c_test.py           # addr 0x28, bus 1
  python3 bno055_i2c_test.py --addr 0x29
  python3 bno055_i2c_test.py --bus 1 --addr 0x28

If you get "No such file or directory: /dev/i2c-1", enable I2C on the Pi and
map /dev/i2c-1 into the container if running in Docker.
"""

import argparse
import time
import sys

try:
    import smbus2
except ImportError:
    print("This script requires smbus2. Install with: pip3 install smbus2")
    sys.exit(1)

# I2C default
DEFAULT_BUS = 1
DEFAULT_ADDR = 0x28

# BNO055 registers (subset)
BNO055_CHIP_ID_ADDR       = 0x00
BNO055_PAGE_ID_ADDR       = 0x07
BNO055_TEMP_ADDR          = 0x34
BNO055_CALIB_STAT_ADDR    = 0x35

BNO055_OPR_MODE_ADDR      = 0x3D
BNO055_PWR_MODE_ADDR      = 0x3E
BNO055_SYS_TRIGGER_ADDR   = 0x3F

# Euler angle registers (unit: 1/16 degree)
BNO055_EUL_HEADING_LSB    = 0x1A  # Heading (yaw)
BNO055_EUL_ROLL_LSB       = 0x1C  # Roll
BNO055_EUL_PITCH_LSB      = 0x1E  # Pitch

# Power modes
POWER_MODE_NORMAL = 0x00

# Operation Modes (Table 3-3 in datasheet)
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF   = 0x0C

# Expected chip ID
EXPECTED_CHIP_ID = 0xA0


def read_u8(bus, addr, reg):
    return bus.read_byte_data(addr, reg)


def write_u8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val)


def read_s16_le(bus, addr, reg):
    """Read little-endian signed 16-bit from reg/reg+1."""
    data = bus.read_i2c_block_data(addr, reg, 2)
    val = data[0] | (data[1] << 8)
    if val & 0x8000:
        val = val - 0x10000
    return val


def set_mode(bus, addr, mode):
    """Switch BNO055 operation mode with required delays."""
    write_u8(bus, addr, BNO055_OPR_MODE_ADDR, mode)
    # Mode switching requires sleep (datasheet suggests up to 30ms)
    time.sleep(0.03)


def reset_device(bus, addr):
    """Perform a soft reset and wait for reboot."""
    write_u8(bus, addr, BNO055_SYS_TRIGGER_ADDR, 0x20)  # RESET
    time.sleep(0.65)  # datasheet suggests ~650ms
    # After reset, chip ID may not be readable immediately
    for _ in range(50):
        try:
            cid = read_u8(bus, addr, BNO055_CHIP_ID_ADDR)
            if cid == EXPECTED_CHIP_ID:
                return True
        except OSError:
            pass
        time.sleep(0.05)
    return False


def init_bno055(bus, addr):
    """Initialize device to NDOF mode."""
    # Read chip ID
    chip_id = read_u8(bus, addr, BNO055_CHIP_ID_ADDR)
    if chip_id != EXPECTED_CHIP_ID:
        # Sometimes after power‑up the ID reads as 0x00 until reset delay
        time.sleep(0.05)
        chip_id = read_u8(bus, addr, BNO055_CHIP_ID_ADDR)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f"Unexpected CHIP ID 0x{chip_id:02X} (expected 0xA0). "
                               f"Check wiring/power/address.")

    # Enter CONFIG mode
    set_mode(bus, addr, OPERATION_MODE_CONFIG)

    # Normal power mode
    write_u8(bus, addr, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
    time.sleep(0.01)

    # Use internal oscillator (clear ext crystal bit in SYS_TRIGGER)
    write_u8(bus, addr, BNO055_SYS_TRIGGER_ADDR, 0x00)
    time.sleep(0.01)

    # Finally set NDOF fusion mode
    set_mode(bus, addr, OPERATION_MODE_NDOF)
    time.sleep(0.05)


def read_euler_deg(bus, addr):
    """Return (heading, roll, pitch) in degrees (float)."""
    h = read_s16_le(bus, addr, BNO055_EUL_HEADING_LSB) / 16.0
    r = read_s16_le(bus, addr, BNO055_EUL_ROLL_LSB) / 16.0
    p = read_s16_le(bus, addr, BNO055_EUL_PITCH_LSB) / 16.0
    return h, r, p


def main():
    parser = argparse.ArgumentParser(description="BNO055 I2C quick test")
    parser.add_argument("--bus", type=int, default=DEFAULT_BUS, help="I2C bus (default 1 on Raspberry Pi)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_ADDR,
                        help="I2C address (default 0x28). Hex allowed, e.g., 0x28 or 0x29")
    parser.add_argument("--samples", type=int, default=10, help="Number of Euler samples to read")
    args = parser.parse_args()

    try:
        bus = smbus2.SMBus(args.bus)
    except FileNotFoundError as e:
        print(f"Cannot open /dev/i2c-{args.bus}: {e}")
        print("Enable I2C on the Raspberry Pi and ensure /dev/i2c-1 exists.")
        return 1


    addr = args.addr    

    try:
        print(f"Probing BNO055 at I2C bus {args.bus}, address 0x{addr:02X} ...")
        # Optional: reset if needed
        # if not reset_device(bus, addr):
        #     print("Warning: soft reset did not confirm chip ID in time.")

        # Initialize
        init_bno055(bus, addr)

        # Basic reads
        chip_id = read_u8(bus, addr, BNO055_CHIP_ID_ADDR)
        page_id = read_u8(bus, addr, BNO055_PAGE_ID_ADDR)
        temp_c  = read_u8(bus, addr, BNO055_TEMP_ADDR)
        calib   = read_u8(bus, addr, BNO055_CALIB_STAT_ADDR)

        print(f"Chip ID: 0x{chip_id:02X} (expected 0xA0)")
        print(f"Page ID: {page_id}")
        print(f"Temperature: {temp_c} °C (sensor internal)")
        print(f"Calibration byte: 0x{calib:02X}  "
              f"(sys:{(calib>>6)&0x03}, gyr:{(calib>>4)&0x03}, acc:{(calib>>2)&0x03}, mag:{calib&0x03})")

        print("\nReading Euler angles (deg):")
        for i in range(args.samples):
            h, r, p = read_euler_deg(bus, addr)
            print(f"{i+1:02d}: heading={h:7.2f}  roll={r:7.2f}  pitch={p:7.2f}")
            time.sleep(0.1)

        print("\nSuccess: Sensor responding over I2C.")
        return 0

    except OSError as e:
        print(f"I2C communication error: {e}")
        print("Check wiring (3.3V, GND, SDA=GPIO2, SCL=GPIO3), address (0x28 vs 0x29), and pull-ups.")
        return 2
    except Exception as e:
        print(f"Error: {e}")
        return 3
    finally:
        try:
            bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
