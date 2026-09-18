#!/usr/bin/env python3
"""实时查看摄像头画面（多摄）— 在 Pi 上运行，本机浏览器打开 http://<pi-ip>:8000

纯 OpenCV + 标准库实现（Pi 无 ffmpeg 时的零依赖方案）：
  - 每个 --device 一个抓图线程，各自编码 JPEG
  - 一个 HTTP 服务：/ 展示全部相机，/stream/<name> 单路 MJPEG

用法:
  python3 scripts/cam_stream.py                                  # 仅 /dev/video0
  python3 scripts/cam_stream.py --device 0 --device 2 --port 8000  # 双摄同页
  python3 scripts/cam_stream.py --device 0 --device 2 --flip video0=v  # video0 上下翻转
"""
from __future__ import annotations

import argparse
import logging
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Condition, Thread

import cv2

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("cam_stream")


class FrameBuffer:
    """单帧缓冲，新帧覆盖旧帧。"""

    def __init__(self) -> None:
        self.cond = Condition()
        self.jpg: bytes | None = None

    def push(self, jpg: bytes) -> None:
        with self.cond:
            self.jpg = jpg
            self.cond.notify_all()


buffers: dict[str, FrameBuffer] = {}
JPEG_QUALITY = 70


def open_camera(device: str, width: int, height: int, fps: int):
    cap = cv2.VideoCapture(device if device.startswith("/") else int(device))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def capture_loop(name: str, device: str, width: int, height: int, fps: int,
                 flip_code: int | None = None) -> None:
    cap = open_camera(device, width, height, fps)
    if not cap.isOpened():
        log.error("[%s] 无法打开摄像头 %s（权限? 设备不存在?）", name, device)
        return
    log.info("[%s] %s 打开成功%s", name, device,
             f"（flip={'vh' if flip_code == -1 else 'v' if flip_code == 0 else 'h'}）"
             if flip_code is not None else "")

    n, fail = 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            fail += 1
            if fail >= 30:
                log.warning("[%s] 连续取帧失败，重开设备", name)
                cap.release()
                time.sleep(1.0)
                cap = open_camera(device, width, height, fps)
                fail = 0
            time.sleep(0.1)
            continue
        fail = 0
        if flip_code is not None:
            frame = cv2.flip(frame, flip_code)
        ok, jpg = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            buffers[name].push(jpg.tobytes())
            n += 1
            if n % 200 == 0:
                log.info("[%s] 已推送 %d 帧", name, n)
        time.sleep(0.05)  # 轻微节流，避免 CPU 拉满


class Handler(BaseHTTPRequestHandler):
    def _html_index(self) -> None:
        parts = []
        for name in buffers:
            parts.append(
                f'<div style="display:inline-block;margin:8px;text-align:center">'
                f'<div style="color:#ccc">{name}</div>'
                f'<img src="/stream/{name}" width="480"></div>')
        body = ("<html><head><title>robohead cameras</title>"
                "<meta http-equiv=\"refresh\" content=\"30\"></head>"
                "<body style=\"margin:0;background:#111\">"
                + "".join(parts) + "</body></html>").encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/":
            self._html_index()
            return
        if self.path.startswith("/stream/"):
            name = self.path[len("/stream/"):]
            buf = buffers.get(name)
            if buf is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with buf.cond:
                        buf.cond.wait(timeout=2.0)
                        jpg = buf.jpg
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 浏览器关闭页面
            return
        self.send_response(404)
        self.end_headers()

    def log_message(self, *args) -> None:
        pass


def main() -> None:
    global JPEG_QUALITY
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", action="append", default=None,
                    help="/dev/videoN 或索引，可多次指定（多摄）")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15,
                    help="推流帧率上限（热点带宽有限，勿设太高）")
    ap.add_argument("--quality", type=int, default=70,
                    help="JPEG 质量 1-100")
    ap.add_argument("--flip", action="append", default=None, metavar="NAME=v|h|vh",
                    help="按相机名翻转: v=上下, h=左右, vh=180°，如 --flip video0=v，可多次指定")
    args = ap.parse_args()
    JPEG_QUALITY = args.quality

    flip_codes = {"v": 0, "h": 1, "vh": -1}
    flips: dict[str, int] = {}
    for spec in args.flip or []:
        name, sep, mode = spec.partition("=")
        if not sep or mode not in flip_codes:
            ap.error(f"--flip 格式应为 相机名=v|h|vh，收到: '{spec}'")
        flips[name] = flip_codes[mode]

    devices = args.device or ["0"]
    for dev in devices:
        name = f"video{dev}" if not dev.startswith("/") else dev.split("/")[-1]
        buffers[name] = FrameBuffer()
        Thread(target=capture_loop,
               args=(name, dev, args.width, args.height, args.fps,
                     flips.get(name)),
               daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    log.info("MJPEG 服务就绪: http://<本机IP>:%d/  相机: %s",
             args.port, ", ".join(buffers))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("退出")


if __name__ == "__main__":
    main()
