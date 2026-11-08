#!/usr/bin/env python3
"""
Can hunt: find Pepsi cans (depth shape + VLM), drive from can to can until no
unvisited can is left in view after a full look around.

  python3 jetson/can_hunt.py --dry-run          look once: detect, measure, verify — no driving
  python3 jetson/can_hunt.py                    hunt (needs Nav2 up: NO_BRIDGE=1 COLOR=true start.sh)

Needs the colour stream (COLOR=true bash jetson/start.sh) and SmolVLM on the Pi.

Per look (robot stopped, coasting finished):
  1. Candidates from DEPTH SHAPE.  Depth pixels → robot frame (calibrated mount,
     floor at --floor-z).  Points 3–20 cm above the floor are grouped into blobs
     (4 cm grid, connected).  A blob is can-shaped if it is ≤ --max-width wide,
     its top is 8–19 cm above the floor, and NOTHING continues above it (chair
     and table legs do).  (COCO YOLOv8n does not detect cans at this size.)
  2. Distance: blob centre (+ can radius along the view ray) → odom position;
     reported from the robot centre and from the camera.
  3. Verify: the blob is projected into the colour image, cropped, and SmolVLM
     on the Pi is asked what brand the drink is (open question: a small VLM
     says "yes" to yes/no questions; it answers "Coca-Cola" for anything it
     can't read).  "Pepsi" → confirmed.  Anything else from farther than
     --verify-range is only "unknown" (label too small to read).
Only cans inside the search area count: --area-ahead m in front of where the
hunt started and --area-side m either side (behind the start never counts).
Then, in order:
  - nearest confirmed, unvisited can → drive to --standoff m short of it,
    facing it → visited.  Legs longer than --direct-range use Nav2 to get within
    it; the last part is a direct slow approach (turn to face, roll forward at
    --approach-speed, stop at the standoff minus coasting), still through the
    Collision Monitor.  Nav2 crawls and stalls on very short goals;
  - else nearest unknown candidate → drive to --inspect m from it and look again
    (the VLM is re-asked from close up; a "no" there rejects it);
  - else turn to look within ±90° of the start heading (45° steps); nothing
    to do after that ends the hunt.
Annotated looks: --out (default /tmp/can_hunt).  Ctrl-C cancels Nav2 goals.
"""
import argparse
import base64
import json
import math
import os
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

CAN_RADIUS = 0.033
GRID = 0.04


def qmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_mat(tf):
    return qmat(tf.rotation), np.array([tf.translation.x, tf.translation.y, tf.translation.z])


def log(msg):
    print(f"[can_hunt {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ── candidates from depth shape ──────────────────────────────────────────────
def blobs(pb, a):
    """pb: N×3 points in base_link.  → list of (points, info) for can-shaped blobs."""
    h = pb[:, 2] - a.floor_z
    rng = np.hypot(pb[:, 0], pb[:, 1])
    inr = (rng > 0.25) & (rng < a.max_range)
    low = pb[inr & (h > 0.03) & (h < 0.20)]
    tall = pb[inr & (h >= 0.20) & (h < 0.80)]
    if len(low) == 0:
        return []
    cells = {}
    for i, k in enumerate(map(tuple, np.floor(low[:, :2] / GRID).astype(int))):
        cells.setdefault(k, []).append(i)
    tall_cells = set(map(tuple, np.floor(tall[:, :2] / GRID).astype(int))) if len(tall) else set()
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        comp, q = [], deque([start]); seen.add(start)
        while q:
            c = q.popleft(); comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in cells and n not in seen:
                        seen.add(n); q.append(n)
        idx = [i for c in comp for i in cells[c]]
        p = low[idx]
        r = float(np.hypot(*p[:, :2].mean(0)))
        ext = np.ptp(p[:, :2], axis=0)
        top = float(np.percentile(p[:, 2] - a.floor_z, 95))
        # nothing above it: no tall points in the blob's cells or their neighbours
        blocked = any((c[0] + dx, c[1] + dy) in tall_cells for c in comp for dx in (-1, 0, 1) for dy in (-1, 0, 1))
        min_pts = max(6, int(40 / max(r, 0.5) ** 2))       # fewer points per can far away
        info = {"n": len(p), "range": r, "width": float(max(ext)), "top": top, "blocked": blocked}
        # depth noise widens small objects with range: allow +3 cm per metre
        info["ok"] = (len(p) >= min_pts and max(ext) <= a.max_width + 0.03 * r and 0.08 <= top <= 0.19
                      and not blocked)
        if info["ok"] or a.verbose:
            out.append((p, info))
    return out


# ── VLM ──────────────────────────────────────────────────────────────────────
def vlm_brand(url, crop, timeout):
    """Ask the VLM what brand the drink is.  → (is_pepsi, answer)."""
    h, w = crop.shape[:2]
    s = 384 / max(h, w)
    crop = cv2.resize(crop, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_CUBIC)
    ok, jpg = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
    body = {"model": "smolvlm-500m", "temperature": 0, "cache_prompt": False, "max_tokens": 12,
            "messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpg).decode()}},
                {"type": "text", "text": "What brand is the drink can in this picture? Answer with the brand name only."}]}]}
    req = urllib.request.Request(url.rstrip("/") + "/v1/chat/completions", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        ans = json.load(r)["choices"][0]["message"]["content"].strip()
    return "pepsi" in ans.lower(), ans


# ── ROS side ─────────────────────────────────────────────────────────────────
class Hunter(Node):
    def __init__(self, a):
        super().__init__("can_hunt")
        self.a = a
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.lock = threading.Lock()
        self.color = self.depth = self.cinfo = self.dinfo = None
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed", self._set("color"), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._set("depth"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._set("cinfo"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._set("dinfo"), qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # through the Collision Monitor, like Nav2's own commands
        self.cmd = self.create_publisher(TwistStamped, "/cmd_vel_nav", 10)
        self.spin_ac = ActionClient(self, Spin, "spin")
        self.active_goal = None

    def _set(self, name):
        def cb(m):
            with self.lock:
                setattr(self, name, m)
        return cb

    def latest(self):
        with self.lock:
            return self.color, self.depth, self.cinfo, self.dinfo

    def robot_pose(self):
        tf = self.buf.lookup_transform("odom", "base_link", Time()).transform
        q = tf.rotation
        return (tf.translation.x, tf.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    # -- actions --------------------------------------------------------------
    def _run_action(self, client, goal, timeout, what):
        if not client.wait_for_server(timeout_sec=5.0):
            log(f"{what}: action server not available"); return False
        fut = client.send_goal_async(goal)
        t0 = time.time()
        while not fut.done():
            time.sleep(0.05)
            if time.time() - t0 > 10:
                log(f"{what}: goal not accepted"); return False
        gh = fut.result()
        if not gh.accepted:
            log(f"{what}: goal rejected"); return False
        self.active_goal = gh
        res = gh.get_result_async()
        while not res.done():
            time.sleep(0.1)
            if time.time() - t0 > timeout:
                log(f"{what}: timed out after {timeout:.0f} s — cancelling")
                self.cancel_active(); return False
        self.active_goal = None
        return res.result().status == GoalStatus.STATUS_SUCCEEDED

    def cancel_active(self):
        gh, self.active_goal = self.active_goal, None
        if gh is not None:
            f = gh.cancel_goal_async()
            t0 = time.time()
            while not f.done() and time.time() - t0 < 3:
                time.sleep(0.05)

    def goto(self, x, y, yaw):
        g = NavigateToPose.Goal()
        p = PoseStamped(); p.header.frame_id = "odom"; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g.pose = p
        return self._run_action(self.nav, g, self.a.nav_timeout, f"goto ({x:.2f},{y:.2f})")

    def drive(self, vx, wz):
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        self.cmd.publish(m)

    def approach(self, tx, ty, stop_at, speed, coast, timeout=25.0):
        """Turn to face (tx, ty), then roll forward until the robot centre is
        stop_at (+ expected coasting) from it.  → True if it got there."""
        t0 = time.time(); phase = "turn"
        try:
            while time.time() - t0 < timeout:
                x, y, yaw = self.robot_pose()
                dist = math.hypot(tx - x, ty - y)
                err = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
                if dist <= stop_at + coast:
                    return True
                if phase == "turn":
                    if abs(err) < math.radians(6):
                        phase = "roll"
                    else:
                        self.drive(0.0, max(-0.6, min(0.6, 1.5 * err)) if abs(err) > 0.25 else 0.4 * math.copysign(1, err))
                if phase == "roll":
                    if abs(err) > math.radians(20):
                        phase = "turn"; continue
                    self.drive(speed, max(-0.4, min(0.4, 1.2 * err)))
                time.sleep(0.05)
            log(f"approach: timed out {dist:.2f} m from the target")
            return False
        finally:
            for _ in range(5):
                self.drive(0.0, 0.0); time.sleep(0.02)

    def turn(self, angle):
        g = Spin.Goal(); g.target_yaw = float(angle)
        g.time_allowance.sec = 20
        return self._run_action(self.spin_ac, g, 30, f"turn {math.degrees(angle):.0f}°")

    # -- perception -----------------------------------------------------------
    def wait_still(self, settle=0.8, timeout=6.0):
        """Wait until base_link stops moving (it coasts ~13 cm), then a little more."""
        t0 = time.time(); last = None; still_since = None
        while time.time() - t0 < timeout:
            try:
                p = self.robot_pose()
            except tf2_ros.TransformException:
                time.sleep(0.1); continue
            if last and math.hypot(p[0] - last[0], p[1] - last[1]) < 0.003 and abs(p[2] - last[2]) < 0.005:
                still_since = still_since or time.time()
                if time.time() - still_since > settle:
                    return True
            else:
                still_since = None
            last = p; time.sleep(0.1)
        return False

    def look(self):
        """→ (bgr image, [det]) from fresh frames.  det: box (colour px), pos (odom), dist, cam_dist, info."""
        t_req = self.get_clock().now()
        while True:
            c, d, ci, di = self.latest()
            if c and d and ci and di and Time.from_msg(c.header.stamp) > t_req and Time.from_msg(d.header.stamp) > t_req:
                break
            time.sleep(0.03)
        img = cv2.imdecode(np.frombuffer(c.data, np.uint8), cv2.IMREAD_COLOR)
        dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
        K = np.array(di.k).reshape(3, 3)
        vs, us = np.mgrid[0:d.height, 0:d.width]
        m = (dep > 0.15) & (dep < self.a.max_range + 1.0)
        z = dep[m]
        P = np.stack([(us[m] - K[0, 2]) / K[0, 0] * z, (vs[m] - K[1, 2]) / K[1, 1] * z, z], 1)
        try:
            Rb, Tb = tf_mat(self.buf.lookup_transform("base_link", di.header.frame_id, Time()).transform)
            Rc, Tc = tf_mat(self.buf.lookup_transform(ci.header.frame_id, "base_link", Time()).transform)
            Ro, To = tf_mat(self.buf.lookup_transform("odom", "base_link", Time.from_msg(d.header.stamp),
                                                      Duration(seconds=0.2)).transform)
        except tf2_ros.TransformException as e:
            log(f"TF not ready: {e}"); return img, []
        pb = P @ Rb.T + Tb
        Kc = np.array(ci.k).reshape(3, 3)
        sx, sy = img.shape[1] / ci.width, img.shape[0] / ci.height
        cam_b = -Rc.T @ Tc                                   # colour camera position in base_link
        out = []
        for p, info in blobs(pb, self.a):
            ctr = p.mean(0)
            ray = ctr[:2] - cam_b[:2]; ray /= np.linalg.norm(ray)
            ctr_b = np.array([ctr[0] + CAN_RADIUS * ray[0], ctr[1] + CAN_RADIUS * ray[1], ctr[2]])
            po = Ro @ ctr_b + To
            # box in the colour image: blob points plus the can's full height
            pts = np.vstack([p, p + [0, 0, 0.02], p * [1, 1, 0] + [0, 0, self.a.floor_z]])
            pc = pts @ Rc.T + Tc
            pc = pc[pc[:, 2] > 0.1]
            u = (Kc[0, 0] * pc[:, 0] / pc[:, 2] + Kc[0, 2]) * sx
            v = (Kc[1, 1] * pc[:, 1] / pc[:, 2] + Kc[1, 2]) * sy
            box = (float(u.min()), float(v.min()), float(u.max()), float(v.max())) if len(u) else None
            out.append({"box": box, "pos": (float(po[0]), float(po[1])),
                        "dist": float(np.hypot(*ctr_b[:2])), "cam_dist": float(np.linalg.norm(ctr - cam_b)),
                        "info": info})
        return img, out


# ── hunt logic ───────────────────────────────────────────────────────────────
class Can:
    _n = 0

    def __init__(self, pos):
        Can._n += 1
        self.id, self.pos, self.hits = Can._n, pos, 0
        self.pepsi = None          # True confirmed, False rejected (close-up no), None unknown
        self.answer = ""; self.asked_at = None
        self.visited = self.failed = self.inspected = False

    def __repr__(self):
        st = "visited" if self.visited else "unreachable" if self.failed else \
            {True: "PEPSI", False: "rejected", None: "unknown"}[self.pepsi]
        return f"can#{self.id} @({self.pos[0]:+.2f},{self.pos[1]:+.2f}) {st} [{self.answer}]"


def annotate(img, dets, path):
    vis = img.copy()
    for d in dets:
        if not d["box"]:
            continue
        x1, y1, x2, y2 = map(int, d["box"])
        c = d.get("can")
        col = (0, 200, 0) if c and c.pepsi else (0, 0, 255) if (c and c.pepsi is False) or not d["info"]["ok"] else (0, 200, 255)
        cv2.rectangle(vis, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), col, 2)
        txt = f"{d['dist']:.2f}m"
        if c:
            txt = f"#{c.id} " + txt + f" {c.answer[:14]}"
        elif not d["info"]["ok"]:
            i = d["info"]; txt += f" x w{i['width']:.2f} top{i['top']:.2f}{' tall' if i['blocked'] else ''}"
        cv2.putText(vis, txt, (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    cv2.imwrite(path, vis)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="one look, no driving")
    ap.add_argument("--verbose", action="store_true", help="also show rejected blobs")
    ap.add_argument("--floor-z", type=float, default=-0.004, help="floor height in base_link (calibrated mount)")
    ap.add_argument("--max-width", type=float, default=0.16, help="m, widest blob still counted as a can")
    ap.add_argument("--max-range", type=float, default=3.5, help="m, ignore candidates further away")
    ap.add_argument("--vlm", default="http://192.168.1.175:8080", help="SmolVLM llama.cpp server")
    ap.add_argument("--vlm-timeout", type=float, default=45.0)
    ap.add_argument("--verify-range", type=float, default=1.5,
                    help="m: a VLM 'no' from closer than this rejects a candidate; from farther it is only unknown")
    ap.add_argument("--standoff", type=float, default=0.5, help="m short of a confirmed can centre to stop")
    ap.add_argument("--area-ahead", type=float, default=5.0, help="m: search area depth in front of the start")
    ap.add_argument("--area-side", type=float, default=2.0, help="m: search area half-width either side")
    ap.add_argument("--direct-range", type=float, default=1.5, help="m: closer than this, approach directly (no Nav2)")
    ap.add_argument("--approach-speed", type=float, default=0.15, help="m/s for the direct approach")
    ap.add_argument("--coast", type=float, default=0.05, help="m the robot rolls after stopping at --approach-speed")
    ap.add_argument("--inspect", type=float, default=1.0, help="m from an unknown candidate to drive for a closer look")
    ap.add_argument("--merge", type=float, default=0.25, help="m, detections this close are the same can")
    ap.add_argument("--nav-timeout", type=float, default=90.0)
    ap.add_argument("--max-cans", type=int, default=12)
    ap.add_argument("--out", default="/tmp/can_hunt")
    a, ros_args = ap.parse_known_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Hunter(a)
    ex = MultiThreadedExecutor(); ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True); spin_thread.start()
    cans: list[Can] = []
    n_look = 0
    start = {}

    def in_area(pos):
        dx, dy = pos[0] - start["x"], pos[1] - start["y"]
        c, s_ = math.cos(start["yaw"]), math.sin(start["yaw"])
        fwd, side = c * dx + s_ * dy, -s_ * dx + c * dy
        return -0.2 <= fwd <= a.area_ahead and abs(side) <= a.area_side

    def do_look():
        nonlocal n_look
        node.wait_still()
        img, dets = node.look()
        n_look += 1
        rx, ry, ryaw = node.robot_pose()
        for d in dets:
            if not d["info"]["ok"]:
                continue
            if not in_area(d["pos"]):
                d["info"]["ok"] = False; d["info"]["outside"] = True
                continue
            c = min(cans, key=lambda c: math.dist(c.pos, d["pos"]), default=None)
            if c is None or math.dist(c.pos, d["pos"]) > a.merge:
                c = Can(d["pos"]); cans.append(c)
            else:   # refine position (running mean)
                c.pos = tuple((np.array(c.pos) * c.hits + d["pos"]) / (c.hits + 1))
            c.hits += 1; d["can"] = c
            ask = c.pepsi is None and (c.asked_at is None or d["dist"] < 0.7 * c.asked_at)
            if ask and d["box"]:
                x1, y1, x2, y2 = d["box"]; w, h = x2 - x1, y2 - y1
                pw, ph = max(0.4 * w, 12), max(0.2 * h, 10)
                crop = img[max(0, int(y1 - ph)):int(y2 + ph), max(0, int(x1 - pw)):int(x2 + pw)]
                try:
                    t0 = time.time()
                    yes, c.answer = vlm_brand(a.vlm, crop, a.vlm_timeout)
                    c.asked_at = d["dist"]
                    if yes:
                        c.pepsi = True
                    elif d["dist"] <= a.verify_range:
                        c.pepsi = False
                    log(f"  VLM on can#{c.id} at {d['dist']:.2f} m: '{c.answer}' → "
                        f"{'PEPSI' if yes else 'rejected' if c.pepsi is False else 'unknown (too far to read)'} "
                        f"({time.time() - t0:.1f} s)")
                except Exception as e:   # noqa: BLE001
                    log(f"  VLM error on can#{c.id}: {e}")
        path = os.path.join(a.out, f"look_{n_look:03d}.jpg")
        annotate(img, dets, path)
        ok = [d for d in dets if d["info"]["ok"]]
        log(f"look {n_look} from ({rx:+.2f},{ry:+.2f}) facing {math.degrees(ryaw):.0f}°: "
            f"{len(ok)} can-shaped object(s) → {path}")
        for d in dets:
            i = d["info"]
            if i["ok"]:
                log(f"  {d['dist']:.2f} m from robot centre ({d['cam_dist']:.2f} m from camera), "
                    f"{i['width'] * 100:.0f} cm wide, {i['top'] * 100:.0f} cm tall → {d['can']}")
            elif i.get("outside"):
                log(f"  (can-shaped object {d['dist']:.2f} m away is outside the search area — ignored)")
            elif a.verbose:
                log(f"  (rejected blob {d['dist']:.2f} m: w {i['width']:.2f} top {i['top']:.2f} n {i['n']}"
                    f"{' has something above' if i['blocked'] else ''})")

    try:
        log(f"waiting for camera, depth and TF (VLM {a.vlm})")
        while True:
            c, d, ci, di = node.latest()
            try:
                node.robot_pose(); ok_tf = True
            except tf2_ros.TransformException:
                ok_tf = False
            if c and d and ci and di and ok_tf:
                break
            time.sleep(0.2)
        start["x"], start["y"], start["yaw"] = node.robot_pose()
        log(f"search area: {a.area_ahead} m ahead of ({start['x']:+.2f},{start['y']:+.2f}) heading "
            f"{math.degrees(start['yaw']):.0f}°, ±{a.area_side} m sideways")
        do_look()
        if a.dry_run:
            log("dry run — known cans: " + ("; ".join(map(str, cans)) or "none"))
            return
        SEARCH = [math.radians(v) for v in (45, 90, -45, -90, 0)]
        search = list(SEARCH)
        while sum(c.visited for c in cans) < a.max_cans:
            rx, ry, _ = node.robot_pose()
            near = lambda cs: min(cs, key=lambda c: math.dist(c.pos, (rx, ry)))
            todo = [c for c in cans if c.pepsi and not c.visited and not c.failed]
            unknown = [c for c in cans if c.pepsi is None and not c.inspected and not c.failed]
            if todo or unknown:
                c = near(todo) if todo else near(unknown)
                stop = a.standoff if todo else a.inspect
                dx, dy = c.pos[0] - rx, c.pos[1] - ry; dist = math.hypot(dx, dy)
                gx, gy = c.pos[0] - stop * dx / dist, c.pos[1] - stop * dy / dist
                what = "can" if todo else "candidate (closer look)"
                log(f"→ {what} {c}: {dist:.2f} m away, driving to ({gx:+.2f},{gy:+.2f}), {stop} m short of it")
                ok = True
                if dist > a.direct_range + 0.2:          # long leg: Nav2 to within direct range
                    k = (dist - a.direct_range) / dist
                    ok = node.goto(rx + k * dx, ry + k * dy, math.atan2(dy, dx))
                    node.wait_still()
                    if not ok:   # Nav2 may still have got close enough
                        x2, y2, _ = node.robot_pose()
                        ok = math.dist((x2, y2), c.pos) < a.direct_range + 0.5
                if ok and math.dist(node.robot_pose()[:2], c.pos) > stop + 0.05:
                    log(f"  direct approach to {stop} m from can#{c.id}")
                    ok = node.approach(c.pos[0], c.pos[1], stop, a.approach_speed, a.coast)
                node.wait_still()
                rx, ry, _ = node.robot_pose()
                if not ok:
                    c.failed = True
                    log(f"✗ could not reach can#{c.id} — skipping it")
                elif todo:
                    c.visited = True
                    log(f"✓ reached can#{c.id}: robot centre now {math.dist(c.pos, (rx, ry)):.2f} m from it "
                        f"({sum(k.visited for k in cans)} visited)")
                else:
                    c.inspected = True
                search = list(SEARCH)
                do_look()
                continue
            if not search:
                break
            target = start["yaw"] + search.pop(0)
            _, _, yaw = node.robot_pose()
            d_yaw = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
            log(f"nothing left in view — turning to {math.degrees(target - start['yaw']):+.0f}° from the start heading")
            node.turn(d_yaw)
            do_look()
        log(f"done: {sum(c.visited for c in cans)} Pepsi can(s) visited")
        for c in cans:
            log(f"  {c}")
    except KeyboardInterrupt:
        log("interrupted — cancelling")
    finally:
        node.cancel_active()
        try:
            node.drive(0.0, 0.0)
        except Exception:   # noqa: BLE001
            pass
        ex.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
