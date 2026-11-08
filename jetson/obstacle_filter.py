#!/usr/bin/env python3
"""
Obstacle filter: depth cloud → clouds Nav2 can trust.

  python3 jetson/obstacle_filter.py            (started by jetson/start.sh)

Why: Nav2's voxel layer judges "obstacle vs floor" by height in the odom
frame, so any odometry height/tilt error lifts the whole floor into a wall.
The depth camera itself is accurate relative to the robot, so this node
removes the floor in the ROBOT frame, before odometry is involved, and then
checks the rest against the robot's own motion before calling it an obstacle.

Per depth frame (throttled to --rate Hz):
  1. Points → base_link (static camera mount), strided and range-cropped.
  2. Floor: fit a plane to near-floor points (RANSAC); fall back to the
     nominal floor if the fit is implausible.  Drop everything within
     --floor-band of it, below it, or above --max-height.
  3. Voxelise what is left (5 cm, ≥ --min-pts points per voxel: drops speckle).
  4. Ego-motion test, per 10 cm odom cell.  A real, static obstacle stays in
     the same odom cell while its position RELATIVE TO THE ROBOT changes
     exactly as the robot moves (its distance shrinks at the robot's speed).
     Anything tied to the robot (pose error, lens artefacts) does not.  So a
     cell is CONFIRMED once it has been seen in ≥ --confirm-hits frames and is
     seen again after its robot-relative position has shifted ≥ --confirm-move
     (from translation or rotation).  Cells unseen for --expire s are dropped.
     Note: the motion test can only judge things the robot has moved relative
     to.  While parked, nothing new gets confirmed (except the close zone).

Publishes (all in base_link, stamped with the source cloud's time):
  /camera/obstacle_points     confirmed cells + everything within --close m
                              → costmap MARKING source
  /camera/obstacle_candidates every voxel that survived the floor cut,
                              confirmed or not → Collision Monitor (slow/approach
                              zones), which acts on raw evidence immediately
  /camera/clearing_points     all points incl. floor, 10 cm voxelised
                              → costmap CLEARING source (much lighter than the
                              raw 270k-point cloud)
Every processed frame publishes all three, empty if need be: the Collision
Monitor stops the robot if its source goes quiet.
"""
import argparse
import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

VOXEL = 0.05        # m, obstacle voxel
CELL = 0.10         # m, odom cell for the ego-motion test
CLEAR_VOXEL = 0.10  # m, clearing cloud voxel


def qmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def voxel_centres(p, size, min_pts=1):
    """Unique voxel centres of points p (N×3), keeping voxels with ≥ min_pts."""
    if len(p) == 0:
        return p
    k = np.floor(p / size).astype(np.int64)
    key = (k[:, 0] + 2**20) * 2**42 + (k[:, 1] + 2**20) * 2**21 + (k[:, 2] + 2**20)
    _, idx, inv, cnt = np.unique(key, return_index=True, return_inverse=True, return_counts=True)
    keep = cnt >= min_pts
    return ((k[idx[keep]] + 0.5) * size)


class ObstacleFilter(Node):
    def __init__(self, a):
        super().__init__("obstacle_filter")
        self.a = a
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.cam_tf = None            # cloud frame → base_link (static)
        self.cells = {}               # (ix, iy) → [first_pose(x,y,yaw), hits, last_t, confirmed]
        self.last_proc = 0.0
        self.floor = None             # ((n, d), fit_ok), refitted every 5th frame
        self.frame_i = 0
        self.stats = {"n": 0, "ms": 0.0, "fit_ok": 0}
        self.pub_obs = self.create_publisher(PointCloud2, "/camera/obstacle_points", 5)
        self.pub_cand = self.create_publisher(PointCloud2, "/camera/obstacle_candidates", 5)
        self.pub_clear = self.create_publisher(PointCloud2, "/camera/clearing_points", 5)
        self.create_subscription(PointCloud2, a.input, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        self.last_report = {}
        self.get_logger().info(
            f"obstacle filter: {a.input} → /camera/obstacle_points (confirmed + <{a.close} m), "
            f"/camera/obstacle_candidates, /camera/clearing_points; floor nominal z={a.floor_z:+.3f} "
            f"band {a.floor_band} m; confirm after {a.confirm_hits} hits + {a.confirm_move} m relative motion")

    # ── helpers ────────────────────────────────────────────────────────────
    def xyz(self, m):
        """Strided N×3 float array from a PointCloud2, NaNs dropped."""
        off = {f.name: f.offset for f in m.fields}
        if not all(c in off for c in "xyz"):
            return np.zeros((0, 3))
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": m.point_step})
        n = m.width * m.height
        s = np.frombuffer(m.data, dtype=dt, count=n)[:: self.a.stride]
        p = np.stack([s["x"], s["y"], s["z"]], axis=1).astype(np.float64)
        return p[np.isfinite(p).all(axis=1)]

    def robot_pose(self, stamp):
        """odom → base_link (x, y, yaw) at stamp, or latest."""
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.buf.lookup_transform("odom", "base_link", t, Duration(seconds=0.05)).transform
                q = tf.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return tf.translation.x, tf.translation.y, yaw
            except tf2_ros.TransformException:
                continue
        return None

    def fit_floor(self, p):
        """Plane (n, d) with n·p + d = height above floor; nominal if the fit is implausible."""
        a = self.a
        nominal = (np.array([0.0, 0.0, 1.0]), -a.floor_z)
        d_h = np.hypot(p[:, 0], p[:, 1])
        c = p[(np.abs(p[:, 2] - a.floor_z) < 0.06) & (d_h < 2.0)]
        if len(c) < 200:
            return nominal, False
        rng = np.random.default_rng()
        c_s = c[rng.choice(len(c), min(len(c), 3000), replace=False)]
        best, best_n = None, 0
        for _ in range(40):
            t = c_s[rng.choice(len(c_s), 3, replace=False)]
            n = np.cross(t[1] - t[0], t[2] - t[0])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n = n / nn
            if n[2] < 0:
                n = -n
            if n[2] < math.cos(math.radians(a.max_tilt)):
                continue
            d = -n @ t[0]
            inl = np.abs(c_s @ n + d) < 0.015
            if inl.sum() > best_n:
                best, best_n = inl, inl.sum()
        if best is None or best_n < 0.3 * len(c_s):
            return nominal, False
        # least-squares refine z = ax + by + c on inliers
        q = c_s[best]
        A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
        (ax, by, cz), *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
        n = np.array([-ax, -by, 1.0]); s = np.linalg.norm(n); n /= s; d = -cz / s
        tilt = math.degrees(math.acos(min(1.0, n[2])))
        if tilt > a.max_tilt or abs(-d - a.floor_z) > 0.05:
            return nominal, False
        return (n, d), True

    def publish(self, pub, header, pts):
        h = type(header)(); h.stamp = header.stamp; h.frame_id = "base_link"
        pub.publish(pc2.create_cloud_xyz32(h, pts.astype(np.float32) if len(pts) else []))

    # ── main callback ──────────────────────────────────────────────────────
    def on_cloud(self, m):
        a = self.a
        now = time.monotonic()
        if now - self.last_proc < 1.0 / a.rate:
            return
        self.last_proc = now
        t0 = time.perf_counter()
        if self.cam_tf is None:
            try:
                tf = self.buf.lookup_transform("base_link", m.header.frame_id, Time(), Duration(seconds=0.1)).transform
                self.cam_tf = (qmat(tf.rotation), np.array([tf.translation.x, tf.translation.y, tf.translation.z]))
            except tf2_ros.TransformException as e:
                self.get_logger().warning(f"waiting for TF base_link ← {m.header.frame_id}: {e}", throttle_duration_sec=5.0)
                return
        pose = self.robot_pose(m.header.stamp)
        R, T = self.cam_tf
        p = self.xyz(m) @ R.T + T
        d_h = np.hypot(p[:, 0], p[:, 1])
        p, d_h = p[d_h < a.clear_range], d_h[d_h < a.clear_range]

        clearing = voxel_centres(p, CLEAR_VOXEL)

        # the floor barely changes between frames: refit every 5th, reuse otherwise
        if self.floor is None or self.frame_i % 5 == 0:
            self.floor = self.fit_floor(p)
        self.frame_i += 1
        (n, d), fit_ok = self.floor
        h = p @ n + d
        keep = (h > a.floor_band) & (h < a.max_height) & (d_h < a.max_range)
        cand = voxel_centres(p[keep], VOXEL, a.min_pts)

        obs = cand[:0]
        if len(cand) and pose is not None:
            rx, ry, ryaw = pose
            cy, sy = math.cos(ryaw), math.sin(ryaw)
            ox = rx + cy * cand[:, 0] - sy * cand[:, 1]
            oy = ry + sy * cand[:, 0] + cy * cand[:, 1]
            ix = np.floor(ox / CELL).astype(np.int64); iy = np.floor(oy / CELL).astype(np.int64)
            ok = np.zeros(len(cand), dtype=bool)
            seen = {}
            for i, key in enumerate(zip(ix.tolist(), iy.tolist())):
                if key in seen:
                    ok[i] = seen[key]; continue
                e = self.cells.get(key)
                if e is None:
                    e = self.cells[key] = [(rx, ry, ryaw), 0, now, False]
                e[1] += 1; e[2] = now
                if not e[3] and e[1] >= a.confirm_hits:
                    # cell centre in the robot frame at first sight vs now
                    cxo, cyo = (key[0] + 0.5) * CELL, (key[1] + 0.5) * CELL
                    fx, fy, fyaw = e[0]
                    def rel(px, py, pyaw):
                        dx, dy = cxo - px, cyo - py
                        return (math.cos(pyaw) * dx + math.sin(pyaw) * dy,
                                -math.sin(pyaw) * dx + math.cos(pyaw) * dy)
                    (ax_, ay_), (bx_, by_) = rel(fx, fy, fyaw), rel(rx, ry, ryaw)
                    if math.hypot(bx_ - ax_, by_ - ay_) >= a.confirm_move:
                        e[3] = True
                seen[key] = ok[i] = e[3]
            close = np.hypot(cand[:, 0], cand[:, 1]) < a.close
            obs = cand[ok | close]
        elif len(cand):
            obs = cand[np.hypot(cand[:, 0], cand[:, 1]) < a.close]   # no pose: close zone only

        # forget cells not seen recently
        if len(self.cells) > 0 and self.stats["n"] % 10 == 0:
            for k in [k for k, e in self.cells.items() if now - e[2] > a.expire]:
                del self.cells[k]

        self.publish(self.pub_obs, m.header, obs)
        self.publish(self.pub_cand, m.header, cand)
        self.publish(self.pub_clear, m.header, clearing)

        s = self.stats
        s["n"] += 1; s["ms"] += (time.perf_counter() - t0) * 1000; s["fit_ok"] += fit_ok
        self.last_report = {"cand": len(cand), "obs": len(obs), "clear": len(clearing),
                            "tilt": math.degrees(math.acos(min(1.0, n[2]))), "floor": -d,
                            "pose": pose is not None}

    def report(self):
        s, r = self.stats, self.last_report
        if not s["n"]:
            self.get_logger().warning(f"no frames processed yet (input {self.a.input})")
            return
        conf = sum(1 for e in self.cells.values() if e[3])
        self.get_logger().info(
            f"{s['n'] / 5.0:.1f} Hz, {s['ms'] / s['n']:.0f} ms/frame, floor fit {100 * s['fit_ok'] / s['n']:.0f}% "
            f"(z {r.get('floor', 0):+.3f}, tilt {r.get('tilt', 0):.1f}°) | voxels: candidates {r.get('cand')}, "
            f"published {r.get('obs')}, clearing {r.get('clear')} | cells tracked {len(self.cells)}, "
            f"confirmed {conf}{'' if r.get('pose') else ' | NO ODOM POSE'}")
        self.stats = {"n": 0, "ms": 0.0, "fit_ok": 0}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", default="/camera/depth/points")
    ap.add_argument("--rate", type=float, default=10.0, help="max frames/s processed")
    ap.add_argument("--stride", type=int, default=3, help="use every Nth point")
    ap.add_argument("--floor-z", type=float, default=-0.004,
                    help="nominal floor height in base_link (camera measured 0.147 m above the floor, "
                         "0.142875 m above base_link)")
    ap.add_argument("--floor-band", type=float, default=0.04, help="m above the floor still counted as floor")
    ap.add_argument("--max-tilt", type=float, default=6.0, help="deg, reject floor fits tilted more")
    ap.add_argument("--max-height", type=float, default=0.60, help="m above floor; higher is ignored")
    ap.add_argument("--max-range", type=float, default=3.0, help="m, obstacles beyond are ignored")
    ap.add_argument("--clear-range", type=float, default=3.5, help="m, clearing cloud range")
    ap.add_argument("--min-pts", type=int, default=3, help="points per 5 cm voxel to count")
    ap.add_argument("--close", type=float, default=0.5, help="m, always-obstacle zone (no confirmation)")
    ap.add_argument("--confirm-hits", type=int, default=3)
    ap.add_argument("--confirm-move", type=float, default=0.10,
                    help="m the cell must shift relative to the robot before it can be confirmed")
    ap.add_argument("--expire", type=float, default=5.0, help="s, forget cells unseen this long")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = ObstacleFilter(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
