#!/usr/bin/env python3
"""
Thermal camera (Hikvision-style USB UVC module) on the Jetson → ROS topics, and
optionally → the Pi's YOLO person detector (start.sh: THERMAL=true).

The module streams 256x392 YUYV frames: rows 0–191 are the camera's own 8-bit
picture (Y channel, its AGC), rows 192–383 the same scene as little-endian
uint16 temperatures (the remaining rows are metadata).  It rejects
VIDIOC_S_FMT (EIO) but defaults to that format, so this node never sets the
format; it reads it (G_FMT) and streams with mmap — plain V4L2 ioctls, no
v4l2-ctl and no OpenCV capture (OpenCV's open sets the format and fails).

Frames come out upright as the module is mounted; --rotate turns them
clockwise (0/90/180/270, default 0) before anything is published.

  topics   /thermal/image_raw/compressed   JPEG, rotated (grey, or --colormap)
           /thermal/temperature            32FC1 °C, rotated (needs the right --encoding)
  YOLO     start.sh with THERMAL=true points jetson/frame_server.py at the JPEG
           topic, so the Pi's person detector (sensors/vision.py) sees thermal
           frames instead of the depth camera's (THERMAL_YOLO=false keeps those).

  python3 jetson/thermal_cam.py --test            one capture, no ROS: PNGs + stats in /tmp/thermal_test
  python3 jetson/thermal_cam.py                   ROS node
  python3 jetson/thermal_cam.py --device /dev/video3 --rotate 270

--device auto picks the first /dev/video* whose current format is 256x392
YUYV.  A missing camera is waited for (re-checked every 3 s), never an exit:
start.sh stops the whole stack when any process exits.  In the privileged
container /dev is populated when the container starts — plug the camera in
before the stack starts (or restart it).

Temperature encoding: unverified.  --test prints both candidates; hold a hand
in view — the right one reads ~30–35 °C on skin.
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import mmap
import os
import select
import struct
import sys
import time

import numpy as np

W, H, IMG_ROWS = 256, 392, 192
YUYV = struct.unpack("<I", b"YUYV")[0]

# ── V4L2 (64-bit layouts) ─────────────────────────────────────────────────────
_IOC = lambda d, n, size: (d << 30) | (size << 16) | (ord("V") << 8) | n
FMT_SIZE, REQ_SIZE, BUF_SIZE = 208, 20, 88
VIDIOC_G_FMT = _IOC(3, 4, FMT_SIZE)
VIDIOC_REQBUFS = _IOC(3, 8, REQ_SIZE)
VIDIOC_QUERYBUF = _IOC(3, 9, BUF_SIZE)
VIDIOC_QBUF = _IOC(3, 15, BUF_SIZE)
VIDIOC_DQBUF = _IOC(3, 17, BUF_SIZE)
VIDIOC_STREAMON = _IOC(1, 18, 4)
VIDIOC_STREAMOFF = _IOC(1, 19, 4)
CAPTURE, MMAP = 1, 1


def get_fmt(fd):
    """→ (width, height, fourcc, sizeimage) of the current capture format."""
    buf = bytearray(FMT_SIZE)
    struct.pack_into("<I", buf, 0, CAPTURE)
    fcntl.ioctl(fd, VIDIOC_G_FMT, buf)
    w, h, pf, _field, _bpl, size = struct.unpack_from("<6I", buf, 8)
    return w, h, pf, size


def _buf(index=0):
    b = bytearray(BUF_SIZE)
    struct.pack_into("<II", b, 0, index, CAPTURE)
    struct.pack_into("<I", b, 60, MMAP)
    return b


class V4L2Capture:
    def __init__(self, dev, nbufs=4):
        self.dev = dev
        self.fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        try:
            self.w, self.h, pf, self.size = get_fmt(self.fd)
            if pf != YUYV:
                raise RuntimeError(f"{dev}: format {struct.pack('<I', pf)!r}, not YUYV")
            req = bytearray(REQ_SIZE)
            struct.pack_into("<III", req, 0, nbufs, CAPTURE, MMAP)
            fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
            n = struct.unpack_from("<I", req, 0)[0]
            self.maps = []
            for i in range(n):
                b = _buf(i)
                fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
                offset = struct.unpack_from("<I", b, 64)[0]
                length = struct.unpack_from("<I", b, 72)[0]
                self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                                           mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))
                fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
            fcntl.ioctl(self.fd, VIDIOC_STREAMON, struct.pack("<I", CAPTURE))
        except Exception:
            os.close(self.fd)
            raise

    def read(self, timeout=2.0):
        """→ (raw uint8 frame bytes, monotonic time) or raises TimeoutError."""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            raise TimeoutError(f"{self.dev}: no frame in {timeout} s")
        b = _buf()
        fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
        t = time.time()
        i, used = struct.unpack_from("<I", b, 0)[0], struct.unpack_from("<I", b, 8)[0]
        data = bytes(self.maps[i][:used or self.size])
        fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
        return data, t

    def close(self):
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, struct.pack("<I", CAPTURE))
        except OSError:
            pass
        for m in self.maps:
            m.close()
        os.close(self.fd)


def find_device():
    """First /dev/video* currently 256x392 YUYV (the thermal module), else None."""
    for dev in sorted(glob.glob("/dev/video*"), key=lambda p: int(p[10:]) if p[10:].isdigit() else 99):
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            w, h, pf, _ = get_fmt(fd)
            if (w, h, pf) == (W, H, YUYV):
                return dev
        except OSError:
            pass
        finally:
            os.close(fd)
    return None


# ── frame decoding ────────────────────────────────────────────────────────────
ENCODINGS = {"kelvin64": lambda r: r / 64.0 - 273.15, "centikelvin": lambda r: r / 100.0 - 273.15}


def split(data, w=W, h=H):
    """Raw YUYV frame → (8-bit picture IMG_ROWS×w, uint16 raw temperatures IMG_ROWS×w)."""
    raw = np.frombuffer(data, np.uint8)[: w * h * 2].reshape(h, w, 2)
    picture = raw[:IMG_ROWS, :, 0].copy()
    temps = np.ascontiguousarray(raw[IMG_ROWS:2 * IMG_ROWS]).view("<u2").reshape(IMG_ROWS, w)
    return picture, temps


def rotate(img, deg_cw):
    """Rotate clockwise by 0/90/180/270 degrees."""
    return np.ascontiguousarray(np.rot90(img, k=-(deg_cw // 90) % 4))


def to_jpeg_image(picture, temps_c, a):
    """The 8-bit image sent to YOLO: the camera's own picture, or temperatures in a fixed range."""
    import cv2
    if a.source == "temp" and temps_c is not None:
        lo, hi = a.temp_range
        g = np.clip((temps_c - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    else:
        g = picture
    if a.colormap != "none":
        return cv2.applyColorMap(g, getattr(cv2, f"COLORMAP_{a.colormap.upper()}"))
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)       # YOLO wants 3 channels


def open_camera(a, log):
    """Block until the camera opens (re-checked every 3 s)."""
    warned = False
    while True:
        dev = a.device if a.device != "auto" else find_device()
        if dev:
            try:
                cap = V4L2Capture(dev)
                log(f"thermal camera {dev}: {cap.w}x{cap.h} YUYV, rotating {a.rotate}° clockwise")
                return cap
            except (OSError, RuntimeError) as e:
                if not warned:
                    log(f"cannot open {dev}: {e} — retrying every 3 s")
        elif not warned:
            log("no thermal camera (no /dev/video* streaming 256x392 YUYV) — waiting, re-checking every 3 s")
        warned = True
        time.sleep(3.0)


def test(a):
    import cv2
    cap = open_camera(a, lambda m: print(m, flush=True))
    try:
        for _ in range(10):                 # let the module's AGC settle
            data, _ = cap.read()
    finally:
        cap.close()
    out = a.out
    os.makedirs(out, exist_ok=True)
    picture, raw = split(data, cap.w, cap.h)
    np.save(f"{out}/raw_u16.npy", raw)
    cv2.imwrite(f"{out}/picture_unrotated.png", picture)
    pr = rotate(picture, a.rotate)
    cv2.imwrite(f"{out}/picture.png", pr)
    cv2.imwrite(f"{out}/picture_inferno.png", cv2.applyColorMap(pr, cv2.COLORMAP_INFERNO))
    cv2.imwrite(f"{out}/yolo_input.jpg", to_jpeg_image(pr, rotate(ENCODINGS[a.encoding](raw.astype(np.float32)),
                                                                     a.rotate), a))
    print(f"picture: {picture.shape[1]}x{picture.shape[0]} → rotated {pr.shape[1]}x{pr.shape[0]}, "
          f"Y {picture.min()}–{picture.max()}")
    print(f"raw uint16: {raw.min()}–{raw.max()}, mean {raw.mean():.1f}")
    for name, f in ENCODINGS.items():
        c = f(raw.astype(np.float64))
        print(f"  {name:12s} min {c.min():6.1f}  max {c.max():6.1f}  mean {c.mean():6.1f} °C"
              + ("   ← --encoding" if name == a.encoding else ""))
    print(f"→ {out}/ (picture.png should be upright; hold a hand in view: the right encoding's max ≈ 30–35 °C)")


def node(a):
    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init()
    n = rclpy.create_node("ghost_thermal")
    log = n.get_logger().info
    pub_jpg = n.create_publisher(CompressedImage, "/thermal/image_raw/compressed", qos_profile_sensor_data)
    pub_t = n.create_publisher(Image, "/thermal/temperature", qos_profile_sensor_data)
    period, last, count, t_log = 1.0 / a.fps, 0.0, 0, time.monotonic()
    conv = ENCODINGS[a.encoding]
    cap = open_camera(a, log)
    try:
        while rclpy.ok():
            try:
                data, t = cap.read()
            except (OSError, TimeoutError) as e:
                log(f"thermal camera lost ({e}) — reopening")
                cap.close() if cap else None
                cap = open_camera(a, log)
                continue
            if time.monotonic() - last < period:
                continue
            last = time.monotonic()
            picture, raw = split(data, cap.w, cap.h)
            temps = rotate(conv(raw.astype(np.float32)), a.rotate)
            picture = rotate(picture, a.rotate)
            stamp = n.get_clock().now().to_msg()
            frame = "thermal_link"
            ok, jpg = cv2.imencode(".jpg", to_jpeg_image(picture, temps, a), [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                m = CompressedImage(format="jpeg", data=jpg.tobytes())
                m.header.stamp, m.header.frame_id = stamp, frame
                pub_jpg.publish(m)
            ti = Image(height=temps.shape[0], width=temps.shape[1], encoding="32FC1",
                       is_bigendian=0, step=temps.shape[1] * 4, data=temps.astype(np.float32).tobytes())
            ti.header.stamp, ti.header.frame_id = stamp, frame
            pub_t.publish(ti)
            count += 1
            if time.monotonic() - t_log > 30:
                log(f"{count} frames; hottest {temps.max():.1f} °C, coldest {temps.min():.1f} °C ({a.encoding})")
                t_log = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        cap.close()
        n.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=os.environ.get("THERMAL_DEVICE", "auto"), help="/dev/videoN or auto")
    ap.add_argument("--rotate", type=int, choices=[0, 90, 180, 270],
                    default=int(os.environ.get("THERMAL_ROTATE", 0)),
                    help="degrees CLOCKWISE applied to every frame (0: the module as mounted gives upright frames)")
    ap.add_argument("--fps", type=float, default=10.0, help="max frames/s published")
    ap.add_argument("--encoding", choices=list(ENCODINGS), default="kelvin64", help="uint16 → °C")
    ap.add_argument("--source", choices=["picture", "temp"], default="picture",
                    help="JPEG / YOLO image: the camera's own picture, or temperatures over --temp-range")
    ap.add_argument("--temp-range", type=lambda s: tuple(float(v) for v in s.split(",")), default=(15.0, 40.0),
                    help="°C 'lo,hi' mapped to black–white with --source temp")
    ap.add_argument("--colormap", default="none", help="none, inferno, jet, … (YOLO usually does best on grey)")
    ap.add_argument("--test", action="store_true", help="one capture without ROS: images + stats")
    ap.add_argument("--out", default="/tmp/thermal_test")
    a = ap.parse_args()
    test(a) if a.test else node(a)


if __name__ == "__main__":
    sys.exit(main())
