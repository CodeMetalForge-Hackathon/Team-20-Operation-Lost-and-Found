"""
IMU sensor — MPU-6050 via I2C (smbus2).
Stub: smooth random-walk that keeps the robot roughly on course.
"""
import math, random, time
from dataclasses import dataclass

@dataclass
class IMUReading:
    ax: float   # body-frame m/s², gravity removed
    ay: float
    gz: float   # yaw rate rad/s
    dt: float   # sample interval s

try:
    import smbus2 as _smbus2
    _HW = True
except ImportError:
    _HW = False

_ADDR        = 0x68
_ACCEL_SCALE = 9.81 / 16384.0   # ±2 g
_GYRO_SCALE  = math.radians(1) / 131.0   # ±250 °/s


class IMUSensor:
    def __init__(self, i2c_bus: int = 1, stub_speed: float = 0.3):
        self._speed    = stub_speed
        self._heading  = 0.0
        self._last_t   = time.monotonic()
        self._hw       = False

        if _HW:
            try:
                self._bus = _smbus2.SMBus(i2c_bus)
                self._bus.write_byte_data(_ADDR, 0x6B, 0x00)   # wake
                self._hw = True
            except Exception as e:
                print(f"[IMU] hw init failed ({e}) — stub mode")

    def read(self) -> IMUReading:
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        return self._read_hw(dt) if self._hw else self._read_stub(dt)

    # ── hardware ────────────────────────────────────────────────────────────

    def _read_hw(self, dt: float) -> IMUReading:
        raw = self._bus.read_i2c_block_data(_ADDR, 0x3B, 14)

        def s16(hi, lo):
            v = (hi << 8) | lo
            return v - 65536 if v > 32767 else v

        ax = s16(raw[0],  raw[1])  * _ACCEL_SCALE
        ay = s16(raw[2],  raw[3])  * _ACCEL_SCALE
        gz = s16(raw[12], raw[13]) * _GYRO_SCALE
        return IMUReading(ax=ax, ay=ay, gz=gz, dt=dt)

    # ── stub ─────────────────────────────────────────────────────────────────

    def _read_stub(self, dt: float) -> IMUReading:
        # Gentle random-walk heading, constant forward speed
        self._heading += random.gauss(0, 0.03)
        v  = self._speed
        ax = math.cos(self._heading) * random.gauss(v, 0.04)
        ay = math.sin(self._heading) * random.gauss(v, 0.04)
        gz = random.gauss(0, 0.01)
        return IMUReading(ax=ax, ay=ay, gz=gz, dt=max(dt, 1e-3))
