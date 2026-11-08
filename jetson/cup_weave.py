#!/usr/bin/env python3
"""
Cup weave: find cups / wine glasses with YOLOv8s (on the Pi's Hailo), weave
between them — passing each on alternating sides — until a PURPLE one ends the
sequence, look for more, and finally drive home back along the waypoints.

  python3 jetson/cup_weave.py --dry-run        one look: detect, colour, distance — no driving
  python3 jetson/cup_weave.py                  run (Nav2 up + colour: COLOR=true, jetson/config/stack.env)

Image processing is all on the Pi: each look's colour frame is POSTed to
sensors/cup_server.py (ghost-cups.service, --detector), which runs YOLOv8s on
the Hailo (whole frame + floor-band tiles) and returns "cup" / "wine glass"
boxes with each box's colour.  The Jetson only adds the depth geometry:

  distance: a ray from the colour camera through the box CENTROID; depth pixels
  that project into the inner box give the distance along it (median), + the
  cup radius → cup centre in odom.  With no depth there — or for a wine glass
  whose depth is much farther than the floor says (seen through it) — the ray
  through the box's bottom-centre is cut with the floor.

Detections within --merge of a known cup are that cup (position: running mean;
colour and kind: votes, closer looks count more).  Detections farther than
--max-range are ignored.

The search goes cup by cup; cups are < 2 m apart, so distance decides which
matter: the FIRST cup is the nearest within --first-range of the robot (the
robot looks around if there is none); after passing a cup, the NEXT is the
nearest unpassed cup within --next-range of it.  Each cup is passed on the
opposite side to the one before (the first on --first-side): the pass point is
--pass-offset beside the cup, across the direction of travel, and --pass-beyond
past it.  The robot drives THROUGH pass points: within --tolerance counts as
passed, and if the next cup is already known its goal goes out at once, so Nav2
blends into the new path instead of stopping.  While driving it looks every
--look-period s (odometry taken at each frame's time), refining cup positions
and finding new cups; the goal follows a refined cup.  It only stops to search
(±45°, ±90°, then behind) when no next cup is known.  After passing a cup, its
coordinate goes into the map with the robot's pose and a colour frame taken
right there.  Only a purple cup ends a sequence: detections within
--purple-radius of it are ignored from then on, and the robot looks for cups
not passed yet within --more-range (all around if none is known); one found
starts a new sequence.  With nothing left it drives home through its pass
waypoints in reverse.  Nav2 plans around boxes on every leg.

Boxes: the obstacle filter's confirmed points (/camera/obstacle_points) are
accumulated on a 5 cm odom grid all run; clusters not explained by a cup are
obstacles ("box" up to --box-max m across, else "large": walls, furniture).
Boxes within --box-near of a cup or of the path between consecutive cups are
logged — when first seen and in the summary — relative to their nearest cup, in
that cup's travel frame (ahead/behind, left/right), and whether they lie
between two consecutive cups; everything else is only in map.json.

Shape cups (OFF by default — pass --vlm http://192.168.1.175:8080 to enable;
without it grey/white cups are ignored and SmolVLM is never called): cups that
are NOT red or blue (--plain-colours) carry a drawn triangle or circle.  YOLO's box of such a cup is cropped tight (10 % margin)
from the full colour frame and SmolVLM on the Pi (--vlm, llama.cpp) is asked
"What shape is drawn on the cup?"; the answer maps to triangle (triangle) or
circle (circle/O/oval/round; square/rectangle count half — that is how SmolVLM
misreads a small circle).  Asks run in a background thread (~6 s each on the
Pi), are repeated when the cup looks --vlm-regrow× bigger than at the last ask
(up to --vlm-asks), and vote weighted by box height.  A cup's CLASS is then
'triangle' / 'circle' (or its colour for red/blue cups, and while undecided);
--colours and --stop-colour take classes as well as colours, e.g.
--colours triangle --stop-colour circle.  White shape cups often read 'grey'
(the --skip-colours default): a skipped colour still counts once the VLM has
named a shape and YOLO has found it --shape-min-dets times (only depth-backed
boxes are asked about: SmolVLM names a shape even on an empty crop).  Searches wait (--vlm-wait) for pending asks before deciding.

Output: --out/run_<time>/: map.json (cups, passes, boxes, path), map.png (top
view), pass_<n>.jpg (frame at each pass), look_<n>.jpg (annotated looks).
Ctrl-C cancels Nav2 goals and stops the wheels.
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
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import can_hunt
from can_hunt import Hunter, tf_mat

CUP_RADIUS = 0.04
BOX_GRID = 0.05
UNSATURATED = {"grey", "white", "clear"}
BGR = {"red": (40, 40, 220), "orange": (0, 140, 255), "yellow": (0, 220, 230), "green": (60, 180, 60),
       "blue": (220, 120, 30), "purple": (180, 50, 150), "pink": (180, 120, 255), "white": (235, 235, 235),
       "clear": (200, 200, 160), "grey": (140, 140, 140)}


def saved_floor_z(default=-0.006):
    """FLOOR_Z from the boot calibration (jetson/config/mount_calibration.env)."""
    try:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "mount_calibration.env")) as fh:
            for line in fh:
                if line.startswith("FLOOR_Z="):
                    return float(line.split("=", 1)[1])
    except (OSError, ValueError):
        pass
    return default


def log(msg):
    print(f"[cup_weave {time.strftime('%H:%M:%S')}] {msg}", flush=True)


can_hunt.log = log          # Hunter's own messages (Nav2 timeouts etc.) under this name


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def detect_remote(url, img, tile, timeout):
    """POST the frame to the Pi's cup server.  → (dets, server ms, round-trip ms)"""
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    t0 = time.time()
    req = urllib.request.Request(f"{url.rstrip('/')}/detect?tile={tile}", data=jpg.tobytes(),
                                 headers={"Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        res = json.load(r)
    return res["dets"], res["ms"], (time.time() - t0) * 1000


class PiLost(Exception):
    """The Pi's detector stopped answering: the weave is over, the robot goes home on odometry."""


# ── shape classification (SmolVLM on the Pi) ─────────────────────────────────
SHAPE_PROMPT = "What shape is drawn on the cup? Answer with one word."
SHAPE_WORDS = [("triangle", "triangle", 1.0), ("circle", "circle", 1.0), ("oval", "circle", 1.0),
               ("ellipse", "circle", 1.0), ("round", "circle", 1.0), ("ring", "circle", 1.0),
               ("square", "circle", 0.5), ("rectangle", "circle", 0.5)]


def shape_crop(img, box, margin=0.1, size=384):
    """Tight crop of a YOLO box (tests: wider crops make SmolVLM call a small circle 'square'), scaled up."""
    x1, y1, x2, y2 = box; w, h = x2 - x1, y2 - y1
    c = img[max(0, int(y1 - margin * h)):int(y2 + margin * h), max(0, int(x1 - margin * w)):int(x2 + margin * w)]
    if c.size == 0:
        return None
    s = size / max(c.shape[:2])
    return cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))), interpolation=cv2.INTER_CUBIC)


def parse_shape(ans):
    """VLM answer → (shape or None, weight)."""
    a = ans.lower().strip().rstrip(".!")
    for word, shape, w in SHAPE_WORDS:
        if word in a:
            return shape, w
    if a in ("o", "0"):
        return "circle", 1.0
    return None, 0.0


def vlm_shape(url, crop, timeout):
    """→ raw answer"""
    ok, jpg = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
    body = {"model": "smolvlm-500m", "temperature": 0, "cache_prompt": False, "max_tokens": 8,
            "messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpg).decode()}},
                {"type": "text", "text": SHAPE_PROMPT}]}]}
    req = urllib.request.Request(url.rstrip("/") + "/v1/chat/completions", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)["choices"][0]["message"]["content"].strip()


class ShapeClassifier:
    """Background SmolVLM asks, one at a time; at most one queued per cup (the newest crop wins)."""

    def __init__(self, a, run_dir):
        self.a, self.run_dir = a, run_dir
        self.cv = threading.Condition()
        self.pending = {}                  # cup id → (cup, crop, box h, dist)
        self.busy = None
        self.n = 0
        threading.Thread(target=self._work, daemon=True).start()

    def eligible(self, c):
        return c.colour not in self.a.plain_colours and not c.passed

    def offer(self, c, img, det):
        """Queue an ask if this view is worth one (eligible cup, big enough, bigger than the last ask)."""
        x1, y1, x2, y2 = det["box"]; h = y2 - y1
        if (not self.eligible(c) or det["colour"] in self.a.plain_colours or h < self.a.vlm_min_px
                or c.asks >= self.a.vlm_asks):
            return
        if not det["method"].startswith("centroid"):
            return          # no depth on the box: often a YOLO hit on nothing, and SmolVLM names a shape anyway
        if c.asked_h and h < self.a.vlm_regrow * c.asked_h:
            return
        crop = shape_crop(img, det["box"])
        if crop is None:
            return
        with self.cv:
            old = self.pending.get(c.id)
            if old is None or h > old[2]:
                self.pending[c.id] = (c, crop, h, det["dist"])
                self.cv.notify_all()

    def idle(self):
        with self.cv:
            return not self.pending and self.busy is None

    def wait(self, timeout):
        """Wait until nothing is queued or being asked.  → True if idle."""
        t_end = time.time() + timeout
        with self.cv:
            while self.pending or self.busy is not None:
                left = t_end - time.time()
                if left <= 0:
                    return False
                self.cv.wait(left)
        return True

    def _work(self):
        while True:
            with self.cv:
                while not self.pending:
                    self.cv.wait()
                cid = max(self.pending, key=lambda k: self.pending[k][2])     # biggest view first
                c, crop, h, dist = self.pending.pop(cid)
                self.busy = cid
            try:
                if c.passed or c.asks >= self.a.vlm_asks or (c.asked_h and h < self.a.vlm_regrow * c.asked_h):
                    continue
                t0 = time.time()
                ans = vlm_shape(self.a.vlm, crop, self.a.vlm_timeout)
                shape, w = parse_shape(ans)
                self.n += 1
                fn = f"vlm_{self.n:03d}_cup{c.id}_{shape or 'none'}.jpg"
                cv2.imwrite(os.path.join(self.run_dir, fn), crop)
                before = c.shape
                c.vote_shape(shape, w * h, ans, h)
                log(f"  VLM cup#{c.id} ({c.colour}, box {h:.0f} px, {dist:.2f} m): '{ans}' → {shape or 'no shape'} "
                    f"({time.time() - t0:.1f} s) votes {c.shape_votes_str()} → class {c.label}"
                    + (" (changed)" if before and before != c.shape else "") + f" [{fn}]")
            except Exception as e:   # noqa: BLE001
                log(f"  VLM error on cup#{c.id}: {e}")
            finally:
                with self.cv:
                    self.busy = None
                    self.cv.notify_all()


# ── map objects ──────────────────────────────────────────────────────────────
class Cup:
    _n = 0

    def __init__(self, pos, kind):
        Cup._n += 1
        self.id, self.pos, self.hits = Cup._n, pos, 0
        self.votes, self.kinds = {}, {kind: 0.0}
        self.passed = self.failed = False
        self.side = None; self.travel = None; self.pass_pose = None; self.frame = None; self.seq = None
        self.shapes, self.answers, self.asks, self.asked_h = {}, [], 0, 0.0
        self.n = 0                            # detections merged into this cup
        self.plain = {"red", "blue"}          # set by main from --plain-colours

    @property
    def shape(self):
        """'triangle' / 'circle' once the VLM's votes say so (shape cups only), else None."""
        if self.colour in self.plain or not self.shapes:
            return None
        s = max(self.shapes, key=self.shapes.get)
        return s if self.shapes[s] > 0 else None

    @property
    def label(self):
        """The cup's class: its shape (shape cups), else its colour."""
        return self.shape or self.colour

    def vote_shape(self, shape, w, ans, h):
        self.asks += 1; self.asked_h = max(self.asked_h, h); self.answers = self.answers + [ans]
        if shape:
            # a new dict, not in place: the main thread reads it while the VLM thread votes
            self.shapes = {**self.shapes, shape: self.shapes.get(shape, 0.0) + w}

    def shape_votes_str(self):
        return ", ".join(f"{k} {v:.0f}" for k, v in self.shapes.items()) or "none"

    def is_(self, name):
        return name in (self.colour, self.shape)

    @property
    def kind(self):
        return max(self.kinds, key=self.kinds.get)

    @property
    def colour(self):
        """Most-voted colour — but a hue the cup clearly showed (≥ 25 % of the votes) beats the
        unsaturated readings: close up a blue cup's shaded side is too dark (V < 45) to have a
        hue and reads grey, and close views weigh most."""
        if not self.votes:
            return "?"
        hued = {k: v for k, v in self.votes.items() if k not in UNSATURATED}
        if hued and max(hued.values()) >= 0.25 * sum(self.votes.values()):
            return max(hued, key=hued.get)
        return max(self.votes, key=self.votes.get)

    def add(self, pos, colour, kind, dist):
        w = 1.0 / max(0.4, dist)
        if self.hits:
            self.pos = tuple((np.array(self.pos) * self.hits + np.array(pos) * w) / (self.hits + w))
            self.hits += w
        else:
            self.pos, self.hits = pos, w
        self.votes[colour] = self.votes.get(colour, 0.0) + w
        self.kinds[kind] = self.kinds.get(kind, 0.0) + w

    def __repr__(self):
        st = "passed" if self.passed else "unreachable" if self.failed else "seen"
        sh = f" [{self.shape}]" if self.shape else ""
        return f"cup#{self.id} {self.colour}{sh} {self.kind} @({self.pos[0]:+.2f},{self.pos[1]:+.2f}) {st}"

    def to_json(self):
        return {"id": self.id, "colour": self.colour, "kind": self.kind, "class": self.label, "shape": self.shape,
                "shape_votes": {k: round(v, 1) for k, v in self.shapes.items()}, "vlm_answers": self.answers, "pos": [round(v, 3) for v in self.pos],
                "votes": {k: round(v, 2) for k, v in self.votes.items()}, "sequence": self.seq,
                "passed": self.passed, "side": self.side, "travel_deg": None if self.travel is None else
                round(math.degrees(self.travel), 1), "pass_pose": self.pass_pose, "frame": self.frame}


# ── ROS side ─────────────────────────────────────────────────────────────────
class Weaver(Hunter):
    def __init__(self, a):
        super().__init__(a)
        self.grid = {}                      # (ix, iy) → [hits, top]
        self.path = []                      # (t, x, y)
        self.create_subscription(PointCloud2, "/camera/obstacle_points", self._on_obstacles, qos_profile_sensor_data)
        self.create_timer(0.5, self._log_pose)

    def nav_start(self, x, y, yaw):
        """Send a Nav2 goal without waiting (a newer goal preempts it).  → result future or None."""
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        if not self.nav.wait_for_server(timeout_sec=5.0):
            log("Nav2 not available"); return None
        g = NavigateToPose.Goal()
        p = PoseStamped(); p.header.frame_id = "odom"; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g.pose = p
        fut = self.nav.send_goal_async(g); t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 10:
                log("Nav2 goal not accepted"); return None
            time.sleep(0.02)
        gh = fut.result()
        if not gh.accepted:
            log("Nav2 goal rejected"); return None
        self.active_goal = gh
        return gh.get_result_async()

    def _log_pose(self):
        try:
            x, y, _ = self.robot_pose()
        except tf2_ros.TransformException:
            return
        if not self.path or math.dist(self.path[-1][1:], (x, y)) > 0.03:
            self.path.append((round(time.time(), 1), round(x, 3), round(y, 3)))

    def _on_obstacles(self, m):
        try:
            R, T = tf_mat(self.buf.lookup_transform("odom", m.header.frame_id, Time.from_msg(m.header.stamp),
                                                    Duration(seconds=0.05)).transform)
        except tf2_ros.TransformException:
            return
        p = np.array([[q[0], q[1], q[2]] for q in pc2.read_points(m, ("x", "y", "z"), skip_nans=True)])
        if len(p) == 0:
            return
        po = p @ R.T + T
        with self.lock:
            for (ix, iy), z in zip(map(tuple, np.floor(po[:, :2] / BOX_GRID).astype(int)), po[:, 2]):
                c = self.grid.setdefault((ix, iy), [0, 0.0]); c[0] += 1; c[1] = max(c[1], float(z))

    def obstacles(self, cups):
        """Clusters of confirmed obstacle cells not explained by a cup."""
        with self.lock:
            cells = {k: v for k, v in self.grid.items() if v[0] >= self.a.box_hits}
        cupxy = [c.pos for c in cups]
        cells = {k: v for k, v in cells.items()
                 if all(math.dist(((k[0] + .5) * BOX_GRID, (k[1] + .5) * BOX_GRID), q) > 0.25 for q in cupxy)}
        seen, out = set(), []
        for s in cells:
            if s in seen:
                continue
            comp, q = [], deque([s]); seen.add(s)
            while q:
                c = q.popleft(); comp.append(c)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (c[0] + dx, c[1] + dy)
                        if n in cells and n not in seen:
                            seen.add(n); q.append(n)
            if len(comp) < self.a.box_cells:
                continue
            xy = (np.array(comp) + 0.5) * BOX_GRID
            lo, hi = xy.min(0) - BOX_GRID / 2, xy.max(0) + BOX_GRID / 2
            size = hi - lo
            out.append({"cells": [tuple(round(float(v), 3) for v in q) for q in xy],
                        "centre": tuple(float(v) for v in xy.mean(0)), "min": tuple(lo.tolist()),
                        "max": tuple(hi.tolist()), "size": tuple(size.tolist()),
                        "height": max(cells[c][1] for c in comp) - self.a.floor_z,
                        "kind": "box" if max(size) <= self.a.box_max else "large"})
        return out

    def look(self):
        """→ (bgr image, [det], timing str): det = kind, score, box, colour, pos (odom), dist, method…"""
        t_req = self.get_clock().now()
        t0 = time.time()
        while True:
            c, d, ci, di = self.latest()
            if c and d and ci and di and Time.from_msg(c.header.stamp) > t_req and Time.from_msg(d.header.stamp) > t_req:
                break
            if time.time() - t0 > 5:
                raise RuntimeError("no fresh colour + depth frames (is the stack up with COLOR=true?)")
            time.sleep(0.03)
        img = cv2.imdecode(np.frombuffer(c.data, np.uint8), cv2.IMREAD_COLOR)
        boxes, srv_ms, rt_ms = detect_remote(self.a.detector, img, self.a.tile, self.a.detector_timeout)
        dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
        K = np.array(di.k).reshape(3, 3)
        vs, us = np.mgrid[0:d.height, 0:d.width]
        m = (dep > 0.15) & (dep < self.a.max_range + 1.0)
        z = dep[m]
        P = np.stack([(us[m] - K[0, 2]) / K[0, 0] * z, (vs[m] - K[1, 2]) / K[1, 1] * z, z], 1)
        Rb, Tb = tf_mat(self.buf.lookup_transform("base_link", di.header.frame_id, Time()).transform)
        Rc, Tc = tf_mat(self.buf.lookup_transform(ci.header.frame_id, "base_link", Time()).transform)
        Ro, To = tf_mat(self.buf.lookup_transform("odom", "base_link", Time.from_msg(c.header.stamp),
                                                  Duration(seconds=0.2)).transform)
        pc = (P @ Rb.T + Tb) @ Rc.T + Tc                          # depth points in the colour camera frame
        pc = pc[pc[:, 2] > 0.1]
        Kc = np.array(ci.k).reshape(3, 3)
        sx, sy = img.shape[1] / ci.width, img.shape[0] / ci.height
        u = (Kc[0, 0] * pc[:, 0] / pc[:, 2] + Kc[0, 2]) * sx
        v = (Kc[1, 1] * pc[:, 1] / pc[:, 2] + Kc[1, 2]) * sy
        cam_b = -Rc.T @ Tc                                        # colour camera in base_link

        def ray_b(px, py):
            r = np.array([(px / sx - Kc[0, 2]) / Kc[0, 0], (py / sy - Kc[1, 2]) / Kc[1, 1], 1.0])
            return r, Rc.T @ r

        out = []
        for b in boxes:
            x1, y1, x2, y2 = b["box"]; w, h = x2 - x1, y2 - y1
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            # depth along the centroid ray: depth pixels landing in the inner box
            inner = (u > cx - 0.25 * w) & (u < cx + 0.25 * w) & (v > cy - 0.3 * h) & (v < cy + 0.3 * h)
            p_depth = p_floor = None
            if inner.sum() >= self.a.min_depth_px:
                zc = float(np.median(pc[inner, 2]))
                rc, _ = ray_b(cx, cy)
                p_depth = Rc.T @ (rc * zc - Tc)                   # centroid point in base_link
            rc, rb = ray_b(cx, y2)                                # bottom-centre ray → floor
            if rb[2] < -1e-3:
                p_floor = cam_b + (self.a.floor_z - cam_b[2]) / rb[2] * rb
            method, pb = None, None
            if p_depth is not None:
                method, pb = "centroid ray + depth", p_depth
                # clear glass: depth may come from what is behind it.  (Not for
                # cups: there the floor ray is the weaker one — a 0.5° pitch
                # error moves it ~20 cm at 2 m.)
                if (b["kind"] == "wine glass" and p_floor is not None
                        and np.hypot(*(p_depth - cam_b)[:2]) > np.hypot(*(p_floor - cam_b)[:2]) + 0.25):
                    method, pb = "floor (depth saw through the glass)", p_floor
            elif p_floor is not None:
                method, pb = "floor (no depth)", p_floor
            if pb is None:
                continue
            hr = pb[:2] - cam_b[:2]; hr /= np.linalg.norm(hr)
            ctr = np.array([pb[0] + CUP_RADIUS * hr[0], pb[1] + CUP_RADIUS * hr[1], 0.0])
            po = Ro @ ctr + To
            rng_ = float(np.hypot(*ctr[:2]))
            if rng_ > self.a.max_range:
                continue
            alt = None if p_floor is None or p_depth is None else float(np.hypot(*(p_floor[:2] - p_depth[:2])))
            out.append({"kind": b["kind"], "score": b["score"], "box": b["box"], "colour": b["colour"],
                        "cinfo": b["cinfo"], "pos": (float(po[0]), float(po[1])), "dist": rng_, "method": method,
                        "depth_vs_floor": alt, "bearing": math.degrees(math.atan2(ctr[1], ctr[0]))})
        return img, out, f"YOLOv8s on the Pi {srv_ms:.0f} ms ({rt_ms:.0f} ms round trip)"


def annotate(img, dets, path, title=""):
    vis = img.copy()
    for d in dets:
        x1, y1, x2, y2 = map(int, d["box"])
        col = BGR.get(d["colour"], (255, 255, 255))
        cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
        c = d.get("cup")
        txt = (f"#{c.id} " if c else "") + f"{d['colour']}" + (f" {c.shape}" if c and c.shape else "") + \
            f" {d['kind']} {d['score']:.2f} {d['dist']:.2f}m"
        cv2.putText(vis, txt, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(vis, txt, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    if title:
        cv2.putText(vis, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(vis, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.imwrite(path, vis)


def weave_distance(box, cups):
    """Closest distance from the obstacle to a cup or to the path between consecutive passed cups."""
    pts = np.array(box["cells"])
    best = min((float(np.min(np.hypot(*(pts - c.pos).T))) for c in cups), default=9e9)
    seqd = sorted([k for k in cups if k.pass_pose], key=lambda k: k.pass_pose["t"])
    for p, q in zip(seqd, seqd[1:]):
        ab = np.subtract(q.pos, p.pos); L2 = float(ab @ ab)
        if L2 < 1e-9:
            continue
        t = np.clip((pts - p.pos) @ ab / L2, 0, 1)
        best = min(best, float(np.min(np.hypot(*(pts - (np.array(p.pos) + t[:, None] * ab)).T))))
    return best


def relative(box, cups):
    """Where an obstacle is relative to the cups (travel frames)."""
    passed = [c for c in cups if c.travel is not None]
    ref = passed or cups
    if not ref:
        return "no cups yet"
    c = min(ref, key=lambda k: math.dist(k.pos, box["centre"]))
    dx, dy = box["centre"][0] - c.pos[0], box["centre"][1] - c.pos[1]
    if c.travel is not None:
        ct, st = math.cos(c.travel), math.sin(c.travel)
        f, l = ct * dx + st * dy, -st * dx + ct * dy
        rel = (f"{abs(f):.2f} m {'ahead of' if f >= 0 else 'behind'} and {abs(l):.2f} m "
               f"{'left' if l >= 0 else 'right'} of cup#{c.id} ({c.colour}, along the weave)")
    else:
        rel = f"{math.hypot(dx, dy):.2f} m from cup#{c.id} ({c.colour}), odom offset ({dx:+.2f},{dy:+.2f})"
    seqd = sorted([k for k in cups if k.pass_pose], key=lambda k: k.pass_pose["t"])
    for p, q in zip(seqd, seqd[1:]):
        ab = np.subtract(q.pos, p.pos); L = np.linalg.norm(ab)
        if L < 1e-6:
            continue
        t = float(np.dot(np.subtract(box["centre"], p.pos), ab) / L ** 2)
        off = abs(float(np.cross(ab, np.subtract(box["centre"], p.pos)) / L))
        if 0 < t < 1 and off < 0.5 * max(box["size"]) + 0.3:
            rel += f"; between cup#{p.id} and cup#{q.id}"
    return rel


def draw_map(path, cups, boxes, robot_path, waypoints, start, unconfirmed=frozenset()):
    pts = [start[:2]] + [c.pos for c in cups] + [p[1:] for p in robot_path] + \
          [b["min"] for b in boxes] + [b["max"] for b in boxes]
    lo = np.min(pts, 0) - 0.5; hi = np.max(pts, 0) + 0.5
    S = 150                                                  # px per metre
    W, H = int((hi[1] - lo[1]) * S) + 1, int((hi[0] - lo[0]) * S) + 1
    W, H = max(W, 300), max(H, 300)
    img = np.full((H, W, 3), 255, np.uint8)
    px = lambda x, y: (int((hi[1] - y) * S), int((hi[0] - x) * S))     # odom x up, y left
    for g in np.arange(math.floor(lo[0]), hi[0], 0.5):
        cv2.line(img, px(g, lo[1]), px(g, hi[1]), (235, 235, 235), 1)
    for g in np.arange(math.floor(lo[1]), hi[1], 0.5):
        cv2.line(img, px(lo[0], g), px(hi[0], g), (235, 235, 235), 1)
    for b in boxes:
        col = (60, 110, 170) if b.get("near_weave") else (200, 200, 200)
        hb = BOX_GRID / 2
        for cx, cy in b["cells"]:
            cv2.rectangle(img, px(cx + hb, cy + hb), px(cx - hb, cy - hb), col, -1)
        if b.get("near_weave"):
            cv2.putText(img, f"box {b['height'] * 100:.0f}cm", px(b["max"][0] + 0.03, b["max"][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
    for p, q in zip(robot_path, robot_path[1:]):
        cv2.line(img, px(*p[1:]), px(*q[1:]), (200, 160, 0), 2)
    for w in waypoints:
        cv2.drawMarker(img, px(*w), (0, 0, 0), cv2.MARKER_TILTED_CROSS, 10, 2)
    for c in cups:
        r = max(4, int(CUP_RADIUS * S))
        if c.id in unconfirmed:                        # probably a YOLO false hit: faint, "?"
            cv2.circle(img, px(*c.pos), r, (190, 190, 190), 1)
            cv2.putText(img, f"#{c.id}?", (px(*c.pos)[0] + 8, px(*c.pos)[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (170, 170, 170), 1)
            continue
        cv2.circle(img, px(*c.pos), r, BGR.get(c.colour, (0, 0, 0)), -1)
        cv2.circle(img, px(*c.pos), r, (0, 0, 0), 1)
        if c.shape:                                    # shape cups: mapped, not weaved — draw the shape
            q = px(*c.pos); k = r + 6
            if c.shape == "triangle":
                cv2.polylines(img, [np.array([(q[0], q[1] - k), (q[0] - k, q[1] + k - 2), (q[0] + k, q[1] + k - 2)])],
                              True, (0, 0, 0), 2)
            else:
                cv2.circle(img, q, k, (0, 0, 0), 2)
        lab = f"#{c.id} {c.colour}" + (f" {c.shape}" if c.shape else "") + (f" ({c.side})" if c.side else "")
        cv2.putText(img, lab, (px(*c.pos)[0] + 8, px(*c.pos)[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.drawMarker(img, px(*start[:2]), (0, 0, 255), cv2.MARKER_STAR, 16, 2)
    cv2.putText(img, "start", (px(*start[:2])[0] + 8, px(*start[:2])[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    cv2.putText(img, "odom: x up, y left, grid 0.5 m", (6, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="one look, no driving")
    ap.add_argument("--detector", default="http://192.168.1.175:8091", help="sensors/cup_server.py on the Pi")
    ap.add_argument("--detector-timeout", type=float, default=5.0)
    ap.add_argument("--tile", type=int, default=480, help="px: YOLO tile size over the floor band (0 = off)")
    ap.add_argument("--min-depth-px", type=int, default=8, help="depth pixels needed in the inner box")
    ap.add_argument("--floor-z", type=float, default=saved_floor_z(),
                    help="floor height in base_link (calibrated: FLOOR_Z in jetson/config/mount_calibration.env)")
    ap.add_argument("--max-range", type=float, default=3.0, help="m, ignore detections farther away")
    ap.add_argument("--first-range", type=float, default=2.5, help="m: the first cup is the nearest within this")
    ap.add_argument("--ahead-fov", type=float, default=100.0,
                    help="deg either side of the direction of travel that counts as 'ahead' for the next cup")
    ap.add_argument("--next-range", type=float, default=2.5, help="m: the next cup is within this of the last one")
    ap.add_argument("--merge", type=float, default=0.35, help="m, detections this close are the same cup")
    ap.add_argument("--first-side", choices=["left", "right"], default="left",
                    help="side of the first cup the robot passes on")
    ap.add_argument("--pass-offset", type=float, default=0.32, help="m, robot centre beside the cup centre")
    ap.add_argument("--pass-beyond", type=float, default=0.15, help="m past the cup along the travel direction")
    ap.add_argument("--tolerance", type=float, default=0.25,
                    help="m: a pass point / waypoint this close counts as reached (the robot drives on)")
    ap.add_argument("--home-tolerance", type=float, default=0.15, help="m: close enough to the start")
    ap.add_argument("--look-period", type=float, default=0.6, help="s between looks while driving")
    ap.add_argument("--carrot", type=float, default=0.0,
                    help="m: Nav2's goal lies this far beyond each pass point / waypoint, so it doesn't brake there")
    ap.add_argument("--resend", type=float, default=0.15, help="m a refined cup must move before its goal is re-sent")
    ap.add_argument("--skip-colours", type=lambda v: set(v.split(",")) if v else set(), default={"grey"},
                    help="comma list: 'cups' of these colours are ignored (YOLO false hits are mostly grey)")
    ap.add_argument("--stop-colour", default="",
                    help="a cup colour that ends a sequence (default none: the weave ends when no next cup is found)")
    ap.add_argument("--colours", type=lambda v: set(v.split(",")) if v else set(), default={"red", "blue"},
                    help="comma list: the cup colours that are weaved (plus --stop-colour).  Every other cup — "
                         "the triangle / circle shape cups — is only classified and mapped, never a target")
    ap.add_argument("--vlm", default="",
                    help="SmolVLM llama.cpp server for shape cups, e.g. http://192.168.1.175:8080 (default '' = off)")
    ap.add_argument("--vlm-timeout", type=float, default=30.0)
    ap.add_argument("--plain-colours", type=lambda v: set(v.split(",")) if v else set(), default={"red", "blue"},
                    help="comma list: cups of these colours have no drawn shape (not sent to the VLM)")
    ap.add_argument("--vlm-min-px", type=float, default=24, help="px: box height a cup needs to be asked about")
    ap.add_argument("--vlm-regrow", type=float, default=1.4,
                    help="ask again once the cup's box is this many times taller than at the last ask")
    ap.add_argument("--vlm-asks", type=int, default=4, help="asks per cup at most")
    ap.add_argument("--shape-min-dets", type=int, default=3,
                    help="detections a grey shape cup needs to count as found (false hits are one-offs); "
                         "fewer → marked unconfirmed in the map")
    ap.add_argument("--pi-fails", type=int, default=2,
                    help="detector calls failing in a row (each up to --detector-timeout) that mean the Pi is gone: "
                         "the weave stops and the robot drives home")
    ap.add_argument("--vlm-wait", type=float, default=15.0,
                    help="s a search (stopped) waits for pending VLM asks before choosing a cup")
    ap.add_argument("--purple-radius", type=float, default=0.5,
                    help="m around a passed stop-colour cup where detections are ignored")
    ap.add_argument("--more-range", type=float, default=3.0,
                    help="m: after a purple cup, look for unpassed cups this far from the robot")
    ap.add_argument("--direct-range", type=float, default=0.6, help="m: Nav2 stopped closer than this → finish directly")
    ap.add_argument("--approach-speed", type=float, default=0.20)
    ap.add_argument("--coast", type=float, default=0.04)
    ap.add_argument("--nav-timeout", type=float, default=90.0)
    ap.add_argument("--max-cups", type=int, default=20)
    ap.add_argument("--box-hits", type=int, default=3, help="confirmed points a 5 cm cell needs")
    ap.add_argument("--box-cells", type=int, default=4, help="cells an obstacle cluster needs")
    ap.add_argument("--box-max", type=float, default=1.0, help="m: clusters wider are 'large' (walls, furniture)")
    ap.add_argument("--box-near", type=float, default=1.0,
                    help="m: boxes this close to a cup or the path between cups are logged")
    ap.add_argument("--out", default="/workspaces/isaac_ros-dev/ghost_runs/cup_weave")
    a, ros_args = ap.parse_known_args()
    run_dir = os.path.join(a.out, time.strftime("run_%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Weaver(a)
    ex = MultiThreadedExecutor(); ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True); spin_thread.start()
    cups: list[Cup] = []
    waypoints = []                    # pass points actually reached, in order
    known_boxes = []
    st = {"look": 0, "seq": 0, "start": None, "pi_fails": 0, "pi_lost": False}
    shapes = ShapeClassifier(a, run_dir) if a.vlm else None

    def save_map():
        boxes = node.obstacles(cups)
        for b in boxes:
            b["relative"] = relative(b, cups)
            b["weave_dist"] = round(weave_distance(b, cups), 3)
            b["near_weave"] = b["kind"] == "box" and b["weave_dist"] <= a.box_near
        with open(os.path.join(run_dir, "map.json"), "w") as fh:
            json.dump({"start": st["start"], "cups": [{**c.to_json(), "detections": c.n, "role": role(c)} for c in cups],
                       "waypoints": waypoints, "pi_lost": st["pi_lost"],
                       "boxes": boxes, "path": node.path}, fh, indent=1)
        try:
            draw_map(os.path.join(run_dir, "map.png"), cups, boxes, node.path, waypoints, st["start"],
                     {c.id for c in cups if not confirmed(c)})
        except Exception as e:   # noqa: BLE001
            log(f"map drawing failed: {e}")
        return boxes

    def report_new_boxes():
        for b in node.obstacles(cups):
            if b["kind"] != "box" or weave_distance(b, cups) > a.box_near:
                continue                          # walls, furniture, things far off the weave
            if any(math.dist(b["centre"], k["centre"]) < 0.3 for k in known_boxes):
                continue
            known_boxes.append(b)
            log(f"  {'BOX' if b['kind'] == 'box' else 'large obstacle'} at ({b['centre'][0]:+.2f},{b['centre'][1]:+.2f}), "
                f"{b['size'][0]:.2f}×{b['size'][1]:.2f} m, {b['height'] * 100:.0f} cm tall: {relative(b, cups)}")

    def wanted(c):
        """Weave targets: cups of --colours (red, blue) and the stop colour.  Shape cups are mapped only."""
        return c.colour in a.colours or (bool(a.stop_colour) and c.is_(a.stop_colour))

    def role(c):
        return "weave" if wanted(c) else "mapped" if confirmed(c) else "unconfirmed"

    def confirmed(c):
        """A shape cup counts as found once YOLO has seen it --shape-min-dets times (false hits are one-offs)."""
        return c.colour not in a.skip_colours or (c.shape is not None and c.n >= a.shape_min_dets)

    def vlm_settle(why):
        """Stopped: give pending shape asks time to finish before a decision."""
        if shapes and not shapes.idle():
            t0 = time.time()
            done = shapes.wait(a.vlm_wait)
            log(f"  {why}: waited {time.time() - t0:.1f} s for the VLM" + ("" if done else " (still busy — going on)"))

    def in_purple_zone(pos):
        return any(k.passed and k.is_(a.stop_colour) and math.dist(k.pos, pos) <= a.purple_radius for k in cups)

    def do_look(tag="", still=True, save="always"):
        """Look (stopped, or on the move: odometry is taken at the frame's time)."""
        if still:
            node.wait_still(settle=0.4)
        while True:
            try:
                img, dets, timing = node.look()
                st["pi_fails"] = 0
                break
            except (OSError, ValueError) as e:        # URLError / timeout / refused / bad JSON
                st["pi_fails"] += 1
                log(f"  detector on the Pi failed ({st['pi_fails']}/{a.pi_fails}): {e!r}")
                if st["pi_fails"] >= a.pi_fails:
                    raise PiLost(repr(e)) from e
                time.sleep(0.5)
        st["look"] += 1
        rx, ry, ryaw = node.robot_pose()
        # skipped colours (grey) stay in when the VLM may name a shape on them
        dets = [d for d in dets if not (d["kind"] == "cup" and d["colour"] in a.skip_colours
                                        and (shapes is None or d["colour"] in a.plain_colours))]
        for d in dets:
            c = min(cups, key=lambda k: math.dist(k.pos, d["pos"]), default=None)
            if c is None or math.dist(c.pos, d["pos"]) > a.merge:
                if in_purple_zone(d["pos"]):         # the purple cup we passed, seen again
                    d["cup"] = None; continue
                c = Cup(d["pos"], d["kind"]); c.plain = a.plain_colours; cups.append(c)
            if not c.passed:            # a passed cup's position is final (it's in the map)
                c.add(d["pos"], d["colour"], d["kind"], d["dist"]); c.n += 1
                if shapes and d["kind"] == "cup":
                    shapes.offer(c, img, d)
            d["cup"] = c
        dets = [d for d in dets if d["cup"] is not None]
        path = None
        if save == "always" or (save == "dets" and dets):
            path = os.path.join(run_dir, f"look_{st['look']:03d}.jpg")
            annotate(img, dets, path, f"look {st['look']} {tag}")
        if dets or still:
            log(f"look {st['look']}{'' if still else ' (moving)'} from ({rx:+.2f},{ry:+.2f}) facing "
                f"{math.degrees(ryaw):.0f}°: {len(dets)} cup(s)/glass(es), {timing}" + (f" → {path}" if path else ""))
        for d in dets:
            chk = "" if d["depth_vs_floor"] is None else f", depth vs floor {d['depth_vs_floor'] * 100:.0f} cm"
            log(f"  {d['kind']} {d['score']:.2f} {d['colour']} ({d['cinfo']}) class {d['cup'].label}: {d['dist']:.2f} m at "
                f"{d['bearing']:+.0f}° [{d['method']}{chk}] → {d['cup']}")
        report_new_boxes()
        return img, dets

    def candidates(near, rng, heading=None):
        """Unpassed, reachable cups within rng of the point near (none in a passed purple cup's
        zone), nearest first.  With a heading: those ahead (±--ahead-fov) first, then the rest —
        so a cup spotted late behind the robot doesn't pull it back and break the chain."""
        cs = [c for c in cups if not c.passed and not c.failed and math.dist(c.pos, near) <= rng
              and not in_purple_zone(c.pos) and wanted(c)]
        cs.sort(key=lambda c: math.dist(c.pos, near))
        if heading is not None:
            ahead = lambda c: abs(wrap(math.atan2(c.pos[1] - near[1], c.pos[0] - near[0]) - heading)) \
                <= math.radians(a.ahead_fov)
            cs = [c for c in cs if ahead(c)] + [c for c in cs if not ahead(c)]
        return cs

    def stop():
        node.cancel_active(); node.wait_still(settle=0.4)

    def search(angles, near, rng, why, heading=None):
        """Stop, turn to each heading (relative to now) and look, until a cup within rng of near shows up."""
        stop()
        _, _, yaw0 = node.robot_pose()
        for ang in angles:
            if candidates(near, rng):
                break
            _, _, yaw = node.robot_pose()
            log(f"{why}: turning to {math.degrees(ang):+.0f}°")
            node.turn(wrap(yaw0 + ang - yaw))
            do_look(f"{why} {math.degrees(ang):+.0f}°")
        c = candidates(near, rng, heading)
        return c[0] if c else None

    def drive(target, tol, what, moving_looks=True, carrot=0.0):
        """Drive toward target() — re-read after every look on the move, the goal follows it —
        until within tol.  Does NOT stop there: the caller sends the next goal (or stops).
        Nav2's goal is carrot m beyond the target along its heading, so the controller
        doesn't brake for a point the robot only drives past.  → (ok, m left)"""
        def goal(x, y, yaw):
            return x + carrot * math.cos(yaw), y + carrot * math.sin(yaw), yaw
        x, y, yaw = target()
        res = node.nav_start(*goal(x, y, yaw))
        t0 = last_look = time.time(); resends = 0
        while time.time() - t0 < a.nav_timeout:
            rx, ry, _ = node.robot_pose()
            left = math.dist((rx, ry), (x, y))
            if left <= tol:
                return True, left
            if res is None or res.done():
                ok = res is not None and res.result().status == 4          # SUCCEEDED
                if left <= tol * 1.5:
                    return True, left
                if ok and resends < 3:
                    # "succeeded" far from the goal: a preempted goal's old path
                    # finished and Nav2 reported it for the new one — send again
                    resends += 1
                    log(f"  {what}: Nav2 says done but {left:.2f} m to go — goal sent again")
                    res = node.nav_start(*goal(x, y, yaw))
                    continue
                if left <= a.direct_range:                                  # Nav2 gave up close by
                    log(f"  {what}: Nav2 stopped {left:.2f} m short — direct finish")
                    node.approach(x, y, 0.0, a.approach_speed, a.coast)
                    rx, ry, _ = node.robot_pose(); left = math.dist((rx, ry), (x, y))
                    return left <= tol * 1.5, left
                return False, left
            if moving_looks and time.time() - last_look >= a.look_period:
                last_look = time.time()
                try:
                    do_look(what, still=False, save="dets")
                except PiLost:
                    raise
                except Exception as e:   # noqa: BLE001
                    log(f"  look on the move failed: {e}")
                nx, ny, nyaw = target()
                if math.dist((nx, ny), (x, y)) > a.resend:                 # cup position refined
                    x, y, yaw = nx, ny, nyaw
                    res = node.nav_start(*goal(x, y, yaw))
            time.sleep(0.05)
        log(f"  {what}: timed out"); stop()
        rx, ry, _ = node.robot_pose()
        return False, math.dist((rx, ry), (x, y))

    def pass_cup(c, side):
        """Drive past c on the given side, without stopping.  → passed?"""
        rx, ry, _ = node.robot_pose()
        origin = (rx, ry)                                   # travel = from here to the cup
        sgn = 1 if side == "left" else -1

        def target():
            travel = math.atan2(c.pos[1] - origin[1], c.pos[0] - origin[0])
            nx, ny = -math.sin(travel) * sgn, math.cos(travel) * sgn
            return (c.pos[0] + a.pass_offset * nx + a.pass_beyond * math.cos(travel),
                    c.pos[1] + a.pass_offset * ny + a.pass_beyond * math.sin(travel), travel)

        px_, py_, travel = target()
        log(f"→ passing cup#{c.id} ({c.label}) on its {side}: via ({px_:+.2f},{py_:+.2f}), "
            f"heading {math.degrees(travel):.0f}°, {math.dist(origin, c.pos):.2f} m away")
        t0 = time.time()
        ok, left = drive(target, a.tolerance, f"pass cup#{c.id}", carrot=a.carrot)
        if not ok:                       # Nav2 gave up (stuck turning, no progress): once more from here
            log(f"  pass cup#{c.id}: Nav2 gave up {left:.2f} m from the pass point — stopping and trying again")
            stop()
            ok, left = drive(target, a.tolerance, f"pass cup#{c.id} (retry)", carrot=a.carrot)
        rx, ry, ryaw = node.robot_pose()
        if not ok:
            c.failed = True
            log(f"✗ could not pass cup#{c.id} ({left:.2f} m from the pass point) — skipping it")
            return False
        _, _, travel = target()
        c.passed, c.side, c.travel, c.seq = True, side, travel, st["seq"]
        c.pass_pose = {"x": round(rx, 3), "y": round(ry, 3), "yaw_deg": round(math.degrees(ryaw), 1), "t": time.time()}
        lost = None
        try:
            img, _ = do_look(f"passed cup#{c.id}", still=False)
            fpath = os.path.join(run_dir, f"pass_{sum(k.passed for k in cups):02d}_cup{c.id}_{c.label}.jpg")
            cv2.imwrite(fpath, img)
            c.frame = os.path.basename(fpath)
        except PiLost as e:
            lost = e
        waypoints.append((round(rx, 3), round(ry, 3)))
        log(f"✓ passed cup#{c.id} ({c.label} {c.kind}) on its {side} in {time.time() - t0:.1f} s: cup at "
            f"({c.pos[0]:+.2f},{c.pos[1]:+.2f}), robot ({rx:+.2f},{ry:+.2f}), {math.dist((rx, ry), c.pos):.2f} m "
            f"from it ({left:.2f} m from the pass point) → {c.frame}")
        save_map()
        if lost:
            raise lost
        return True

    try:
        log(f"waiting for colour, depth and TF (detector {a.detector}, floor z {a.floor_z:+.4f})")
        try:
            with urllib.request.urlopen(f"{a.detector.rstrip('/')}/health", timeout=a.detector_timeout) as r:
                h = json.load(r)
        except (OSError, ValueError) as e:
            log(f"detector on the Pi not reachable ({e!r}) — not starting"); return
        log(f"detector: {h.get('backend')} {os.path.basename(h.get('model', ''))}")
        t0 = time.time()
        while True:
            c_, d_, ci, di = node.latest()
            try:
                node.robot_pose(); ok_tf = True
            except tf2_ros.TransformException:
                ok_tf = False
            if c_ and d_ and ci and di and ok_tf:
                break
            if time.time() - t0 > 20 and not c_:
                log("no colour frames — start the stack with COLOR=true"); return
            time.sleep(0.2)
        sx, sy, syaw = node.robot_pose()
        st["start"] = (round(sx, 3), round(sy, 3), round(math.degrees(syaw), 1))
        log(f"start ({sx:+.2f},{sy:+.2f}) facing {math.degrees(syaw):.0f}° → output {run_dir}")
        t_run = time.time()
        try:
            do_look("start")
            first = candidates((sx, sy), a.first_range)
            if a.dry_run:
                save_map()
                log("dry run — cups: " + ("; ".join(map(str, cups)) or "none"))
                log(f"first cup would be: {first[0] if first else f'none within {a.first_range} m'}")
                return

            side = a.first_side
            AROUND = [math.radians(v) for v in (60, 120, 180, 240, 300)]
            NEAR = [math.radians(v) for v in (45, 90, -45, -90)]
            BACK = [math.radians(v) for v in (135, 180, -135)]
            rx, ry, _ = node.robot_pose()
            c = first[0] if first else search(AROUND, (rx, ry), a.first_range, "looking for a first cup")
            while c is not None and sum(k.passed for k in cups) < a.max_cups:
                st["seq"] += 1
                log(f"=== sequence {st['seq']}: starts at {c}")
                last, purple = None, False
                while c is not None:                     # one sequence: only a purple cup ends it
                    passed = pass_cup(c, side)
                    side = "right" if side == "left" else "left"     # the weave pattern holds even past a skipped cup
                    if passed:
                        if c.is_(a.stop_colour):
                            last = c
                            log(f"=== sequence {st['seq']} ends at {a.stop_colour} cup#{c.id} ({c.label}) — ignoring {a.purple_radius} m "
                                f"around it from now on")
                            purple = True
                            break
                    last = c       # passed or skipped: the chain goes on from this cup's position
                    ref = last.pos
                    travel = last.travel if last.travel is not None else node.robot_pose()[2]
                    nxt = candidates(ref, a.next_range, travel)
                    if nxt:                              # known already: straight on, no stop
                        c = nxt[0]; continue
                    c = search(NEAR + BACK, ref, a.next_range, f"next cup within {a.next_range} m", travel)
                    if c is None:
                        log(f"=== no {a.stop_colour} cup, and nothing within {a.next_range} m all around — finishing")
                if not purple:
                    break
                # after a purple cup: any cup not found yet, within --more-range of here?
                rx, ry, _ = node.robot_pose()
                nxt = candidates((rx, ry), a.more_range)
                c = nxt[0] if nxt else search(AROUND, (rx, ry), a.more_range, f"looking for more cups within {a.more_range} m")
                if c is None:
                    log("no more cups")
        except PiLost as e:
            st["pi_lost"] = True
            log(f"✗ the Pi stopped answering ({e}) — abandoning the weave, returning home on odometry")
            if a.dry_run:
                return
            stop()

        # home along the waypoints, newest first — through them, no stops
        route = [w for w in reversed(waypoints[:-1])] + [(sx, sy)]
        log(f"→ returning home via {len(route) - 1} waypoint(s)")
        for i, (wx, wy) in enumerate(route):
            rx, ry, _ = node.robot_pose()
            last_leg = i == len(route) - 1
            if not last_leg and math.dist((rx, ry), (wx, wy)) < a.tolerance + 0.15:
                continue
            nxt_w = route[i + 1] if not last_leg else None
            yaw = syaw if last_leg else math.atan2(nxt_w[1] - wy, nxt_w[0] - wx)   # on toward the next one
            tol = a.home_tolerance if last_leg else a.tolerance
            ok, left = drive(lambda: (wx, wy, yaw), tol, "home" if last_leg else f"waypoint {len(route) - 1 - i}",
                             moving_looks=False, carrot=0.0 if last_leg else a.carrot)
            log(f"  {'home' if last_leg else 'waypoint'} ({wx:+.2f},{wy:+.2f}): {'ok' if ok else 'NOT reached'}, {left:.2f} m off")
        stop()
        rx, ry, ryaw = node.robot_pose()
        log(f"home: ({rx:+.3f},{ry:+.3f}) facing {math.degrees(ryaw):.0f}°, {math.dist((rx, ry), (sx, sy)):.2f} m "
            f"from the start — run took {time.time() - t_run:.0f} s")
    except KeyboardInterrupt:
        log("interrupted — cancelling")
    finally:
        node.cancel_active()
        try:
            node.drive(0.0, 0.0)
        except Exception:   # noqa: BLE001
            pass
        try:
            if st["start"]:
                if shapes and not st["pi_lost"]:
                    shapes.wait(a.vlm_wait)          # pending shape answers make it into the map
                boxes = save_map()
                log(f"summary — {sum(c.passed for c in cups)} cup(s) passed in {st['seq']} sequence(s):")
                for c in sorted(cups, key=lambda k: (k.pass_pose or {}).get("t", 1e12)):
                    tag = "" if wanted(c) else " — mapped only" if confirmed(c) else \
                        f" — unconfirmed (seen {c.n}×, likely a false hit)"
                    log(f"  {c}{tag}" + (f", passed on its {c.side} in sequence {c.seq}" if c.passed else ""))
                far = [b for b in boxes if not b["near_weave"]]
                boxes = [b for b in boxes if b["near_weave"]]
                log(f"boxes within {a.box_near} m of the weave: {len(boxes)}"
                    f" ({len(far)} other obstacle cluster(s) — walls, furniture, far things — in map.json only)")
                for b in boxes:
                    log(f"  {b['kind']} at ({b['centre'][0]:+.2f},{b['centre'][1]:+.2f}), "
                        f"{b['size'][0]:.2f}×{b['size'][1]:.2f} m, {b['height'] * 100:.0f} cm tall: {b['relative']}")
                log(f"map → {run_dir}/map.json, map.png")
        except Exception as e:   # noqa: BLE001
            log(f"could not save the map: {e}")
        ex.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
