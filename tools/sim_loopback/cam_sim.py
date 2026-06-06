#!/usr/bin/env python3
"""
Fake-sim camera UDP sender. Emits chunked JPEG frames mimicking the
AI Grand Prix simulator's vision stream on port 5600.

Each frame is a synthetic 640x360 BGR image with a moving white circle
and a frame counter so you can visually verify in rqt_image_view that
frames are arriving in order and unbroken.

Header layout (24 bytes, little-endian, matches §4.6):
  frame_id        uint32  unique sequence id for the frame
  chunk_id        uint16  index of this packet within the frame
  total_chunks    uint16  total packets required to assemble the frame
  jpeg_size       uint32  size of the complete reconstructed JPEG
  payload_size    uint32  size of the JPEG slice in this packet
  sim_time_ns     uint64  simulator capture timestamp, nanoseconds

Usage:
  python3 cam_sim.py                       # localhost, 30 Hz
  python3 cam_sim.py --host 10.0.0.42      # remote bridge
  python3 cam_sim.py --fps 15              # half rate for debugging
  python3 cam_sim.py --chunk-bytes 800     # smaller chunks (more fragmentation)
"""

import argparse
import math
import socket
import struct
import time

import cv2
import numpy as np


HEADER_FORMAT = "<IHHIIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
assert HEADER_SIZE == 24

WIDTH = 640
HEIGHT = 360


def make_frame(frame_id: int, t: float) -> np.ndarray:
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    xs = np.linspace(0, 255, WIDTH, dtype=np.uint8)
    img[:, :, 0] = xs[None, :]
    ys = np.linspace(0, 255, HEIGHT, dtype=np.uint8)
    img[:, :, 2] = ys[:, None]

    cx = int(WIDTH / 2 + 0.4 * WIDTH * math.cos(t))
    cy = int(HEIGHT / 2 + 0.4 * HEIGHT * math.sin(t))
    cv2.circle(img, (cx, cy), 30, (255, 255, 255), -1)

    #Frame_id overlay so missing frames are visible at a glance.
    cv2.putText(img, f"frame {frame_id}  t={t:6.2f}s",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return img


def send_frame(sock: socket.socket, dest, frame_id: int, sim_time_ns: int,
               jpeg: bytes, chunk_bytes: int):
    jpeg_size = len(jpeg)
    total_chunks = (jpeg_size + chunk_bytes - 1) // chunk_bytes
    if total_chunks > 65535:
        raise ValueError(
            f"Too many chunks ({total_chunks}); increase --chunk-bytes"
        )

    for chunk_id in range(total_chunks):
        start = chunk_id * chunk_bytes
        payload = jpeg[start:start + chunk_bytes]
        header = struct.pack(
            HEADER_FORMAT,
            frame_id,
            chunk_id,
            total_chunks,
            jpeg_size,
            len(payload),
            sim_time_ns,
        )
        sock.sendto(header + payload, dest)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5600)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--jpeg-quality", type=int, default=80)
    p.add_argument("--chunk-bytes", type=int, default=1200,
                   help="Payload bytes per UDP packet (header is +24)")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    dest = (args.host, args.port)

    period = 1.0 / args.fps
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]

    print(f"Sending {args.fps:.0f} fps to {dest}, "
          f"chunks={args.chunk_bytes}B + {HEADER_SIZE}B header")

    start = time.monotonic()
    sim_t0_ns = time.time_ns()
    next_send = start
    frame_id = 0
    bytes_sent = 0

    try:
        while True:
            now = time.monotonic()
            if now < next_send:
                time.sleep(max(0.0, next_send - now))
            t = time.monotonic() - start
            sim_time_ns = sim_t0_ns + int(t * 1_000_000_000)

            img = make_frame(frame_id, t)
            ok, jpeg = cv2.imencode(".jpg", img, encode_params)
            if not ok:
                raise RuntimeError("JPEG encode failed")
            jpeg_bytes = jpeg.tobytes()

            send_frame(sock, dest, frame_id, sim_time_ns,
                       jpeg_bytes, args.chunk_bytes)
            bytes_sent += len(jpeg_bytes)
            frame_id += 1
            next_send += period

            if frame_id % 30 == 0:
                elapsed = t if t > 0 else 1e-9
                kbps = (bytes_sent * 8 / 1000) / elapsed
                print(f"sent {frame_id} frames  "
                      f"last_jpeg={len(jpeg_bytes)}B  ~{kbps:.0f} kbps")

    except KeyboardInterrupt:
        print(f"\nstopped at frame {frame_id}")


if __name__ == "__main__":
    main()
