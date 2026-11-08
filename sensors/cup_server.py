#!/usr/bin/env python3
"""
Cup detector service: YOLOv8s on the Hailo (shared with fusion.py through
hailort_service) for jetson/cup_weave.py.  All its image processing is here;
the Jetson only does the depth geometry.

  python3 sensors/cup_server.py --port 8091        (systemd: sensors/ghost-cups.service)

  POST /detect   body: JPEG.   ?tile=480 (px, 0 = whole frame only)
     → {"ms": …, "w": …, "h": …, "model": …,
        "dets": [{"kind": "cup"|"wine glass", "score": …, "box": [x1, y1, x2, y2] (px),
                  "colour": "red"|…|"purple"|…, "hue": …, "cinfo": "…"}]}
  GET  /health   → {"ok": true, "backend": "hailo", "model": …}

Detection: the whole frame plus overlapping tile×tile crops of the floor band
(rows 30–92 %): small, low cups are only found zoomed in.  Boxes are merged
across tiles (IoU), a cup and a wine glass on the same object keep the better.
Colour: dominant hue of the saturated pixels in the inner part of the box;
few saturated pixels → white / clear (wine glass) / grey.
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from detector import PersonDetector

COCO = {40: "wine glass", 41: "cup"}


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


def dominant_hue(hues):
    hist = np.bincount(np.asarray(hues, int), minlength=180).astype(float)[:180]
    hist = np.convolve(np.r_[hist[-4:], hist, hist[:4]], np.ones(9) / 9, "valid")    # circular smooth
    return int(np.argmax(hist))


def hue_name(hue, purple):
    if purple[0] <= hue <= purple[1]:
        return "purple"
    if hue < 8 or hue >= 170:
        return "red"
    if hue < 20:
        return "orange"
    if hue < 34:
        return "yellow"
    if hue < 85:
        return "green"
    if hue < purple[0]:
        return "blue"
    return "pink"


def box_colour(img, box, kind, purple, sat_min):
    """→ (name, hue or None, info)"""
    x1, y1, x2, y2 = box; w, h = x2 - x1, y2 - y1
    crop = img[int(y1 + 0.15 * h):int(y2 - 0.25 * h), int(x1 + 0.22 * w):int(x2 - 0.22 * w)]
    if crop.size == 0:
        return "grey", None, "empty crop"
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(int)
    sat = hsv[(hsv[:, 1] >= sat_min) & (hsv[:, 2] >= 45)]
    frac = len(sat) / len(hsv)
    if frac < 0.18:
        v = float(np.median(hsv[:, 2]))
        return ("clear" if kind == "wine glass" else "white" if v > 150 else "grey"), None, \
            f"{frac:.0%} saturated, V {v:.0f}"
    hue = dominant_hue(sat[:, 0])
    return hue_name(hue, purple), hue, f"hue {hue}, {frac:.0%} saturated"


class Cups:
    def __init__(self, a):
        self.a = a
        self.det = PersonDetector(conf=a.conf)
        if not self.det.available:
            raise SystemExit("no detector backend (Hailo / ONNX)")
        self.lock = threading.Lock()           # one inference at a time

    def run(self, img):
        h, w = img.shape[:2]
        with self.lock:
            out = [(c, s, (b[0] * w, b[1] * h, b[2] * w, b[3] * h)) for c, s, b in self.det.detect_classes(img, COCO)]
        return out

    def detect(self, img, tile):
        H, W = img.shape[:2]
        dets = self.run(img)
        if tile:
            y0, y1 = int(0.30 * H), int(0.92 * H)
            for x0 in sorted(set(list(range(0, max(1, W - tile + 1), tile * 2 // 3)) + [max(0, W - tile)])):
                for c, s, (a, b, c2, d) in self.run(img[y0:y1, x0:x0 + tile]):
                    dets.append((c, s, (a + x0, b + y0, c2 + x0, d + y0)))
        dets.sort(key=lambda d: -d[1]); keep = []
        for d in dets:
            if all(iou(d[2], k[2]) < 0.4 for k in keep):
                keep.append(d)
        res = []
        for c, s, box in keep:
            if (box[2] - box[0]) * (box[3] - box[1]) < self.a.min_box_px:
                continue
            kind = COCO[c]
            colour, hue, info = box_colour(img, box, kind, self.a.purple_hue, self.a.sat_min)
            res.append({"kind": kind, "score": round(s, 3), "box": [round(v, 1) for v in box],
                        "colour": colour, "hue": hue, "cinfo": info})
        return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--conf", type=float, default=0.30, help="YOLO score threshold")
    ap.add_argument("--tile", type=int, default=480, help="default tile size, px (0 = whole frame only)")
    ap.add_argument("--min-box-px", type=float, default=300, help="px², smaller boxes dropped")
    ap.add_argument("--purple-hue", type=lambda s: tuple(int(v) for v in s.split(",")), default=(135, 170),
                    help="OpenCV hue range (0–179) counted as purple, 'lo,hi' (the purple cups are magenta-ish, hue 155–161)")
    ap.add_argument("--sat-min", type=int, default=70, help="HSV saturation a coloured pixel needs")
    a = ap.parse_args()
    cups = Cups(a)
    print(f"[cups] {cups.det.backend} {cups.det.model} on http://{a.host}:{a.port}/detect", flush=True)

    class H(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            if urlparse(self.path).path == "/health":
                self._send(200, {"ok": True, "backend": cups.det.backend, "model": str(cups.det.model)})
            else:
                self._send(404, {"error": "GET /health or POST /detect"})

        def do_POST(self):
            u = urlparse(self.path)
            if u.path != "/detect":
                return self._send(404, {"error": "POST /detect"})
            t0 = time.time()
            n = int(self.headers.get("Content-Length", 0))
            img = cv2.imdecode(np.frombuffer(self.rfile.read(n), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return self._send(400, {"error": "body is not an image"})
            tile = int(parse_qs(u.query).get("tile", [a.tile])[0])
            try:
                dets = cups.detect(img, tile)
            except Exception as e:   # noqa: BLE001
                return self._send(500, {"error": repr(e)})
            self._send(200, {"ms": round((time.time() - t0) * 1000, 1), "w": img.shape[1], "h": img.shape[0],
                             "model": str(cups.det.model).rsplit("/", 1)[-1], "dets": dets})

        def log_message(self, *args):
            pass

    try:
        ThreadingHTTPServer((a.host, a.port), H).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cups.det.close()


if __name__ == "__main__":
    main()
