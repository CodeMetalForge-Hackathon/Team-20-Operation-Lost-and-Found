#!/usr/bin/env python3
"""
Calibrate the camera's yaw on the robot: run the wheels straight forward, then
straight back, for a fixed time at a fixed speed (pure body-x commands, no
steering, no Nav2) and watch which way cuVSLAM's odometry says we went.

  python3 jetson/calibrate_yaw.py                        1.5 s at 0.15 m/s, out and back, 3 reps
  python3 jetson/calibrate_yaw.py --time 2 --speed 0.2 --reps 5

cuVSLAM's base_link heading is really the camera's heading pushed through the
static mount TF.  If the camera is yawed by γ on the robot and the TF doesn't
know it, a straight drive shows up in odom at an angle α = −γ off the reported
heading.  Each leg's motion is summed step by step in the heading reported at
that step (so the robot curving on uneven wheels doesn't bias it), whole leg
incl. the coast, and over the cruise only (first/last --trim m skipped: the
start-up kick yaws and shoves the base sideways).  The back leg is flipped so
both legs should agree.  The odometry is printed while driving; raw poses go
to jetson/logs/calibrate_yaw.json.

  legs agree       → camera yaw: re-launch with CAM_YAW=<suggested> (deg)
  legs disagree    → the wheels crab/curve (mecanum slip), not the camera

The suggestion adds to the yaw already in the TF, so a re-run after applying it
should read ~0°.  Commands go straight to the base driver (/cmd_vel) — no
Collision Monitor — so leave ~(time × speed + 0.1) m clear ahead.  Needs
camera + odometry + wheels only:  NO_NAV2=1 bash jetson/start.sh
(with Nav2 up, the Collision Monitor also writes /cmd_vel).  Ctrl-C stops the wheels.
"""
import argparse
import json
import os
import math
import statistics
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time


LOG_NAME = "calibrate_yaw"   # calibrate_mount.py relabels the shared helpers' output


def log(msg):
    print(f"[{LOG_NAME} {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def circ_mean(angles):
    return math.atan2(sum(map(math.sin, angles)), sum(map(math.cos, angles)))


class Calib(Node):
    def __init__(self, topic):
        super().__init__("calibrate_yaw")
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.cmd = self.create_publisher(TwistStamped, topic, 10)

    def pose(self):
        tf = self.buf.lookup_transform("odom", "base_link", Time()).transform
        return tf.translation.x, tf.translation.y, yaw_of(tf.rotation)

    def mount_yaw(self):
        """Yaw already in the base_link → camera_link TF, rad (None if absent)."""
        try:
            return yaw_of(self.buf.lookup_transform("base_link", "camera_link", Time()).transform.rotation)
        except tf2_ros.TransformException:
            return None

    def drive(self, vx):
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "base_link"
        m.twist.linear.x = float(vx)
        self.cmd.publish(m)

    def stop(self):
        for _ in range(5):
            self.drive(0.0); time.sleep(0.02)

    def wait_still(self, settle=0.8, timeout=6.0, samples=None):
        t0 = time.time(); last = None; since = None
        while time.time() - t0 < timeout:
            p = self.pose()
            if samples is not None:
                samples.append(p)
            if last and math.hypot(p[0] - last[0], p[1] - last[1]) < 0.002 and abs(p[2] - last[2]) < 0.004:
                since = since or time.time()
                if time.time() - since > settle:
                    return p
            else:
                since = None
            last = p; self.drive(0.0); time.sleep(0.1)
        return self.pose()

    def leg(self, vx, duration, name):
        """Command vx (body x only, no steering) for `duration` s, then stop
        and settle, printing the odometry (in the start pose's frame) at
        ~5 Hz.  → (samples, start, end)"""
        start = self.pose(); samples = [start]; t0 = time.time(); shown = -1
        c, sn = math.cos(start[2]), math.sin(start[2])

        def show(p, tag):
            dx, dy = p[0] - start[0], p[1] - start[1]
            log(f"  {name} {time.time() - t0:4.1f}s {tag:5s} fwd {(c * dx + sn * dy) * 100:+6.1f} cm  "
                f"left {(-sn * dx + c * dy) * 100:+5.1f} cm  yaw {math.degrees(wrap(p[2] - start[2])):+5.1f}°")
        try:
            while time.time() - t0 < duration:
                p = self.pose(); samples.append(p)
                if int((time.time() - t0) * 5) > shown:
                    shown = int((time.time() - t0) * 5); show(p, "drive")
                self.drive(vx); time.sleep(0.02)
        finally:
            self.stop()
        end = self.wait_still(samples=samples)
        samples.append(end); show(end, "still")
        return samples, start, end


def leg_angle(samples, start, end, sign, trim=0.0):
    """Direction of the leg's motion in the robot's own (reported) frame, rad:
    each step's displacement rotated by the heading at that step, summed over
    the part of the leg between `trim` m from either end (sign −1 flips a
    backward leg onto the forward direction).
    → (angle, straight-line distance, heading change over the cruise)"""
    steps = []; prog = 0.0
    for (x0, y0, h0), (x1, y1, h1) in zip(samples, samples[1:]):
        dx, dy = x1 - x0, y1 - y0
        h = circ_mean([h0, h1])
        f, l = sign * (math.cos(h) * dx + math.sin(h) * dy), sign * (-math.sin(h) * dx + math.cos(h) * dy)
        prog += f; steps.append((prog, f, l, h1))
    if trim > 0 and prog > 3 * trim:
        steps = [st for st in steps if trim <= st[0] <= prog - trim]
    fx, fy = sum(st[1] for st in steps), sum(st[2] for st in steps)
    return (math.atan2(fy, fx), math.hypot(end[0] - start[0], end[1] - start[1]),
            wrap(steps[-1][3] - steps[0][3]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--time", type=float, default=1.5, help="s of driving per leg")
    ap.add_argument("--speed", type=float, default=0.15, help="m/s, fixed")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--trim", type=float, default=0.05, help="m skipped at each end for the cruise figure")
    ap.add_argument("--topic", default="/cmd_vel", help="the base driver directly (no Collision Monitor)")
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Calib(a.topic)
    ex = MultiThreadedExecutor(); ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True); spin.start()
    res = {"fwd": [], "back": []}; raw = []; mount = None
    try:
        while True:
            try:
                node.pose(); break
            except tf2_ros.TransformException:
                time.sleep(0.2)
        time.sleep(0.5)
        mount = node.mount_yaw()
        log(f"mount TF yaw now {math.degrees(mount or 0.0):+.2f}°; {a.reps}× {a.time:.1f} s at {a.speed:.2f} m/s "
            f"on {a.topic}, out and back")
        node.wait_still()
        for r in range(1, a.reps + 1):
            for d_name, sign in (("fwd", +1), ("back", -1)):
                s, p0, p1 = node.leg(sign * a.speed, a.time, f"rep {r} {d_name:4s}")
                raw.append({"rep": r, "dir": d_name, "poses": s})
                whole, dist, dyaw = leg_angle(s, p0, p1, sign)
                cruise, _, cyaw = leg_angle(s, p0, p1, sign, a.trim)
                res[d_name].append((whole, cruise))
                log(f"rep {r} {d_name:4s}: {dist:.3f} m, {math.degrees(whole):+.2f}° off heading "
                    f"(cruise {math.degrees(cruise):+.2f}°), turned {math.degrees(dyaw):+.2f}°")
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        node.stop()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "calibrate_yaw.json")
    try:
        with open(out, "w") as fh:
            json.dump({"args": vars(a), "mount_yaw": mount, "legs": raw}, fh)
        log(f"raw poses → {out}")
    except (OSError, NameError) as e:
        log(f"could not save raw poses: {e}")
    if res["fwd"] and res["back"]:
        for i, what in ((0, "whole legs"), (1, "cruise only")):
            fw, bk = [x[i] for x in res["fwd"]], [x[i] for x in res["back"]]
            log(f"{what:11s}: forward {math.degrees(circ_mean(fw)):+.2f}°, back {math.degrees(circ_mean(bk)):+.2f}°, "
                f"all {math.degrees(circ_mean(fw + bk)):+.2f}° (spread "
                f"{statistics.pstdev([math.degrees(x) for x in fw + bk]):.2f}°)")
        fwd = [x[1] for x in res["fwd"]]; back = [x[1] for x in res["back"]]
        f, b = math.degrees(circ_mean(fwd)), math.degrees(circ_mean(back))
        alpha = math.degrees(circ_mean(fwd + back))
        if abs(f - b) > max(1.0, 0.5 * abs(alpha)):
            log("forward and back disagree: part of this is the wheels crabbing/curving, not the camera — "
                "check wheel slip / signs before trusting the yaw")
        cam_yaw = math.degrees(mount or 0.0) - alpha
        log(f"camera points {abs(alpha):.2f}° {'right' if alpha > 0 else 'left'} of robot forward → "
            f"suggest CAM_YAW={cam_yaw:.1f} (deg, ROS sign, + = camera turned left)")
    ex.shutdown(timeout_sec=2.0)
    spin.join(timeout=2.0)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
