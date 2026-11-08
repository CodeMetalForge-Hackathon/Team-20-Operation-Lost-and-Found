#!/usr/bin/env python3
"""
Drive to a point in the odom frame, reliably, and optionally face a heading.

  python3 jetson/go_to.py 0.15 -0.16            go to (0.15, -0.16)
  python3 jetson/go_to.py 0.15 -0.16 --yaw 3    ... then face 3° (odom)
  python3 jetson/go_to.py 0 0                   back to the launch point

Nav2 does the long leg (it plans around obstacles); if it stops short — it
crawls and gives up on the last few tens of cm — a direct slow approach
finishes the job, still through the Collision Monitor (/cmd_vel_nav).
Needs the stack up with Nav2 (e.g. NO_BRIDGE=1 bash jetson/start.sh).
Ctrl-C cancels the Nav2 goal and stops the robot.
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from can_hunt import Hunter, log


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--yaw", type=float, default=None, help="deg (odom) to face at the end")
    ap.add_argument("--tolerance", type=float, default=0.06, help="m: close enough, no direct finish")
    ap.add_argument("--direct-max", type=float, default=0.6,
                    help="m: finish directly only if Nav2 left us closer than this")
    ap.add_argument("--approach-speed", type=float, default=0.12)
    ap.add_argument("--nav-timeout", type=float, default=90.0)
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Hunter(argparse.Namespace(nav_timeout=a.nav_timeout))
    ex = MultiThreadedExecutor(); ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True); spin.start()
    t0 = time.time()
    try:
        while True:                                   # wait for TF
            try:
                x, y, _ = node.robot_pose(); break
            except Exception:   # noqa: BLE001
                time.sleep(0.2)
        left = lambda: math.hypot(a.x - x, a.y - y)
        log(f"from ({x:+.2f},{y:+.2f}): {left():.2f} m to ({a.x:+.2f},{a.y:+.2f})")
        if left() > 0.5:
            yaw = math.radians(a.yaw) if a.yaw is not None else math.atan2(a.y - y, a.x - x)
            ok = node.goto(a.x, a.y, yaw)
            node.wait_still(); x, y, _ = node.robot_pose()
            log(f"Nav2 {'succeeded' if ok else 'stopped'} at ({x:+.2f},{y:+.2f}), {left():.2f} m left")
        if a.tolerance < left() <= max(a.direct_max, 0.5):
            log(f"direct finish ({left():.2f} m)")
            node.approach(a.x, a.y, 0.0, a.approach_speed, 0.04)
            node.wait_still(); x, y, _ = node.robot_pose()
        if a.yaw is not None:
            _, _, yaw = node.robot_pose()
            d = math.atan2(math.sin(math.radians(a.yaw) - yaw), math.cos(math.radians(a.yaw) - yaw))
            if abs(d) > math.radians(5):
                node.turn(d); node.wait_still()
        x, y, yaw = node.robot_pose()
        log(f"{'arrived' if left() <= 0.15 else 'NOT arrived'}: ({x:+.3f},{y:+.3f}) facing {math.degrees(yaw):.0f}°, "
            f"{left():.3f} m from the target ({time.time() - t0:.0f} s)")
    except KeyboardInterrupt:
        log("interrupted — cancelling")
    finally:
        node.cancel_active()
        try:
            node.drive(0.0, 0.0)
        except Exception:   # noqa: BLE001
            pass
        ex.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
