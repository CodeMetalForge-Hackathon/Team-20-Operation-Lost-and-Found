#!/usr/bin/env python3
"""
Start-up camera-mount calibration: pitch, roll and floor height from the floor
plane (standing still), then yaw from a short straight drive out and back.

  python3 jetson/calibrate_mount.py --out jetson/config/mount_calibration.env
  python3 jetson/calibrate_mount.py --no-drive        floor only, wheels untouched

Run by jetson/start.sh on every boot (CALIBRATE=0 skips it) with only the
camera, cuVSLAM and the base driver up: commands go straight to /cmd_vel, no
Nav2, no Collision Monitor.

Floor: the depth cloud is put into base_link through the CURRENT mount TF and a
plane is fitted (obstacle_filter.py's RANSAC fit) over --floor-frames frames.
With the mount right, the floor's normal is base_link's z; whatever rotation
takes the measured normal onto z is the mount error, and the corrected mount
is that rotation applied to the current one.  The floor's height in base_link
comes out too (→ obstacle_filter.py --floor-z).

Yaw: the drive test from calibrate_yaw.py (fixed-speed legs, fwd then back).
Before driving it checks the depth cloud for anything standing in the drive
box ahead and skips the drive if so.

Each part is applied only if it passes its quality checks; otherwise the value
the stack was started with is kept.  The env file is written either way, with
CAM_PITCH, CAM_ROLL, CAM_YAW (deg, ROS sign) and FLOOR_Z (m).  Exit 0 = file
written, 1 = nothing usable (no TF / cloud).
"""
import argparse
import math
import statistics
import threading
import time
from types import SimpleNamespace

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

import calibrate_yaw
from calibrate_yaw import Calib, circ_mean, leg_angle, log
from obstacle_filter import ObstacleFilter, qmat

calibrate_yaw.LOG_NAME = "calibrate_mount"


def rpy_of(R):
    """ROS fixed-axis roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll)), rad."""
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))),
            math.atan2(R[1, 0], R[0, 0]))


def rot_between(a, b):
    """Smallest rotation taking unit vector a onto unit vector b."""
    v = np.cross(a, b); s = np.linalg.norm(v); c = float(a @ b)
    if s < 1e-12:
        return np.eye(3)
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


class MountCalib(Calib):
    def __init__(self, a):
        super().__init__("/cmd_vel")
        self.a = a
        self.fa = SimpleNamespace(stride=a.stride, floor_z=a.floor_z, max_tilt=a.max_tilt)
        self.lock = threading.Lock()
        self.cloud = None; self.cloud_n = 0
        self.create_subscription(PointCloud2, a.input, self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, m):
        with self.lock:
            self.cloud = m; self.cloud_n += 1

    def next_cloud(self, timeout=5.0):
        with self.lock:
            n0 = self.cloud_n
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.cloud_n > n0:
                    return self.cloud
            time.sleep(0.02)
        return None

    def mount_R(self):
        tf = self.buf.lookup_transform("base_link", "camera_link", Time()).transform
        return qmat(tf.rotation)

    def cloud_in_base(self, m):
        """Cloud → base_link through the current TF chain (incl. the mount)."""
        tf = self.buf.lookup_transform("base_link", m.header.frame_id, Time()).transform
        R = qmat(tf.rotation); T = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        return ObstacleFilter.xyz(SimpleNamespace(a=self.fa), m) @ R.T + T

    def fit(self, p):
        return ObstacleFilter.fit_floor(SimpleNamespace(a=self.fa), p)


def calibrate_floor(node, a):
    """→ (corrected mount R or None, floor_z or None, info str)"""
    normals, heights, fails = [], [], 0
    for _ in range(a.floor_frames):
        m = node.next_cloud()
        if m is None:
            return None, None, "no depth cloud"
        (n, d), ok = node.fit(node.cloud_in_base(m))
        if ok:
            normals.append(n); heights.append(-d)
        else:
            fails += 1
    if len(normals) < 0.6 * a.floor_frames:
        return None, None, f"floor fit failed in {fails}/{a.floor_frames} frames (floor not clear in view?)"
    n = np.mean(normals, axis=0); n /= np.linalg.norm(n)
    tilts = [math.degrees(math.acos(min(1.0, float(x @ n)))) for x in normals]
    jitter = statistics.pstdev(tilts) if len(tilts) > 1 else 0.0
    err = math.degrees(math.acos(min(1.0, n[2])))
    info = (f"floor normal {err:.2f}° off base_link z over {len(normals)} frames (jitter {jitter:.2f}°), "
            f"floor z {statistics.mean(heights):+.4f} m (±{statistics.pstdev(heights) * 1000:.1f} mm)")
    if jitter > a.max_jitter:
        return None, None, info + f" — jitter > {a.max_jitter}°, not applied"
    C = rot_between(n, np.array([0.0, 0.0, 1.0]))
    return C @ node.mount_R(), statistics.mean(heights), info


def path_blocked(node, a, floor_z):
    """Anything above the floor in the box ahead we're about to drive into?"""
    m = node.next_cloud()
    if m is None:
        return "no depth cloud"
    p = node.cloud_in_base(m)
    h = p[:, 2] - floor_z
    box = (p[:, 0] > 0.05) & (p[:, 0] < a.clear_ahead) & (np.abs(p[:, 1]) < a.clear_half_width) \
        & (h > 0.04) & (h < 0.6)
    k = int(box.sum())
    if k > a.block_pts:
        near = float(p[box, 0].min())
        return f"{k} points above the floor in the drive box, nearest {near:.2f} m ahead"
    return None


def calibrate_yaw(node, a, mount_yaw):
    """→ (new yaw rad or None, info str)"""
    res = {"fwd": [], "back": []}; bad = []
    for r in range(1, a.reps + 1):
        for name, sign in (("fwd", +1), ("back", -1)):
            s, p0, p1 = node.leg(sign * a.speed, a.time, f"rep {r} {name:4s}")
            ang, dist, dyaw = leg_angle(s, p0, p1, sign)
            res[name].append(ang)
            log(f"rep {r} {name:4s}: {dist:.3f} m, {math.degrees(ang):+.2f}° off heading, "
                f"turned {math.degrees(dyaw):+.2f}°")
            if dist < 0.5 * a.speed * a.time:
                bad.append(f"rep {r} {name} only {dist:.3f} m (wheels weak / blocked?)")
            if abs(math.degrees(dyaw)) > a.max_turn:
                bad.append(f"rep {r} {name} turned {math.degrees(dyaw):+.1f}°")
    allv = res["fwd"] + res["back"]
    alpha = circ_mean(allv)
    f, b = math.degrees(circ_mean(res["fwd"])), math.degrees(circ_mean(res["back"]))
    spread = statistics.pstdev([math.degrees(x) for x in allv])
    info = f"fwd {f:+.2f}°, back {b:+.2f}°, all {math.degrees(alpha):+.2f}° (spread {spread:.2f}°)"
    if spread > a.max_spread:
        bad.append(f"spread {spread:.2f}° > {a.max_spread}°")
    if abs(f - b) > a.max_disagree:
        bad.append(f"fwd/back disagree by {abs(f - b):.2f}° (wheels crabbing?)")
    if bad:
        return None, info + " — not applied: " + "; ".join(bad)
    return mount_yaw - alpha, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="env file to write (default: print only)")
    ap.add_argument("--no-drive", action="store_true", help="floor only; keep the current yaw")
    ap.add_argument("--input", default="/camera/depth/points")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--floor-z", type=float, default=-0.004, help="m, expected floor in base_link (fit sanity)")
    ap.add_argument("--max-tilt", type=float, default=6.0, help="deg, largest floor tilt believed")
    ap.add_argument("--floor-frames", type=int, default=20)
    ap.add_argument("--max-jitter", type=float, default=0.5, help="deg, floor normal frame-to-frame")
    ap.add_argument("--time", type=float, default=1.5, help="s per drive leg")
    ap.add_argument("--speed", type=float, default=0.15, help="m/s")
    ap.add_argument("--reps", type=int, default=4, help="out-and-back strokes; more = less noise")
    ap.add_argument("--max-turn", type=float, default=3.0, help="deg per leg, else wheels suspect")
    ap.add_argument("--max-spread", type=float, default=2.5, help="deg across legs")
    ap.add_argument("--max-disagree", type=float, default=3.0, help="deg between fwd and back means")
    ap.add_argument("--clear-ahead", type=float, default=0.60, help="m of floor that must be clear ahead")
    ap.add_argument("--clear-half-width", type=float, default=0.22, help="m")
    ap.add_argument("--block-pts", type=int, default=40, help="points above the floor that count as blocked")
    ap.add_argument("--wait", type=float, default=60.0, help="s to wait for odometry + cloud")
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = MountCalib(a)
    ex = MultiThreadedExecutor(); ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True); spin.start()
    code = 1
    try:
        t0 = time.time()
        while True:
            try:
                node.pose(); R0 = node.mount_R()
                if node.next_cloud(1.0) is not None:
                    break
            except tf2_ros.TransformException:
                pass
            if time.time() - t0 > a.wait:
                log(f"no odometry / mount TF / depth cloud after {a.wait:.0f} s — giving up"); return 1
            time.sleep(0.5)
        node.wait_still(timeout=3.0)
        roll0, pitch0, yaw0 = rpy_of(R0)
        log(f"current mount: pitch {math.degrees(pitch0):+.2f}°, roll {math.degrees(roll0):+.2f}°, "
            f"yaw {math.degrees(yaw0):+.2f}°")

        roll, pitch, floor_z = roll0, pitch0, None
        Rn, fz, info = calibrate_floor(node, a)
        log(f"floor: {info}")
        if Rn is not None:
            roll, pitch, _ = rpy_of(Rn); floor_z = fz
            log(f"floor: pitch {math.degrees(pitch0):+.2f}° → {math.degrees(pitch):+.2f}°, "
                f"roll {math.degrees(roll0):+.2f}° → {math.degrees(roll):+.2f}°")

        yaw = yaw0
        if a.no_drive:
            log("yaw: --no-drive, kept")
        else:
            blocked = path_blocked(node, a, floor_z if floor_z is not None else a.floor_z)
            if blocked:
                log(f"yaw: not driving — {blocked}; kept {math.degrees(yaw0):+.2f}°")
            else:
                y, info = calibrate_yaw(node, a, yaw0)
                log(f"yaw: {info}")
                if y is not None:
                    yaw = y
                    log(f"yaw: {math.degrees(yaw0):+.2f}° → {math.degrees(yaw):+.2f}°")

        lines = [f"# camera mount, written by jetson/calibrate_mount.py {time.strftime('%Y-%m-%d %H:%M:%S')}",
                 f"CAM_PITCH={math.degrees(pitch):.2f}",
                 f"CAM_ROLL={math.degrees(roll):.2f}",
                 f"CAM_YAW={math.degrees(yaw):.2f}"]
        if floor_z is not None:
            lines.append(f"FLOOR_Z={floor_z:.4f}")
        log("result: " + "  ".join(lines[1:]))
        if a.out:
            with open(a.out, "w") as fh:
                fh.write("\n".join(lines) + "\n")
            log(f"→ {a.out}")
        code = 0
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        node.stop()
        ex.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
