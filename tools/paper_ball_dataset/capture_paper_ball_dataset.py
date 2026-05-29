#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def detect_white_paper_ball(img: np.ndarray, min_area: float = 250.0) -> tuple[int, int, int, int] | None:
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    boxes: list[tuple[float, int, int, int, int]] = []
    for s_max, v_min in ((55, 145), (70, 140), (85, 135)):
        mask = ((hsv[:, :, 1] < s_max) & (hsv[:, :, 2] > v_min)).astype(np.uint8) * 255

        # Robot camera is mounted low; patrol trash should be on the floor.
        # Keep enough vertical range for far targets, but strongly score lower
        # contours so chair/table highlights do not win.
        mask[: int(h * 0.25), :] = 0
        mask[:, : int(w * 0.04)] = 0
        mask[:, int(w * 0.98) :] = 0

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > 85000:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 12 or bh < 10:
                continue
            aspect = bw / max(1, bh)
            if aspect < 0.20 or aspect > 5.2:
                continue
            cx = x + bw * 0.5
            cy = y + bh * 0.5
            if cy < h * 0.38:
                continue
            center_bias = 1.0 - min(1.0, abs(cx - w * 0.52) / (w * 0.60))
            lower_bias = clamp((cy - h * 0.32) / (h * 0.58), 0.0, 1.25)
            size_bias = clamp((bw * bh) / (w * h * 0.12), 0.15, 1.2)
            score = area * (0.35 + center_bias) * (0.45 + lower_bias**2) * (0.50 + size_bias)
            boxes.append((score, x, y, bw, bh))

    if not boxes:
        return None

    boxes.sort(reverse=True)
    # Merge nearby white fragments of the same paper ball. Keep this bounded:
    # floor reflections can create many close fragments, and this script must
    # never block field data capture.
    _, x, y, bw, bh = boxes[0]
    base = np.array([x, y, x + bw, y + bh], dtype=np.float32)
    bx0, by0, bx1, by1 = base
    bcx = (bx0 + bx1) * 0.5
    bcy = (by0 + by1) * 0.5
    for _, ox, oy, ow, oh in boxes[1:25]:
        ocx = ox + ow * 0.5
        ocy = oy + oh * 0.5
        if abs(ocx - bcx) < max(55, (bx1 - bx0) * 1.0) and abs(ocy - bcy) < max(65, (by1 - by0) * 1.1):
            bx0 = min(bx0, ox)
            by0 = min(by0, oy)
            bx1 = max(bx1, ox + ow)
            by1 = max(by1, oy + oh)
    base = np.array([bx0, by0, bx1, by1], dtype=np.float32)

    x0, y0, x1, y1 = [float(v) for v in base]
    pad_x = max(8.0, (x1 - x0) * 0.18)
    pad_y = max(8.0, (y1 - y0) * 0.22)
    x0 = clamp(x0 - pad_x, 0, w - 1)
    y0 = clamp(y0 - pad_y, 0, h - 1)
    x1 = clamp(x1 + pad_x, 1, w)
    y1 = clamp(y1 + pad_y, 1, h)
    if x1 <= x0 or y1 <= y0:
        return None
    return int(x0), int(y0), int(x1 - x0), int(y1 - y0)


def write_yolo_label(path: Path, bbox: tuple[int, int, int, int], image_w: int, image_h: int) -> None:
    x, y, w, h = bbox
    cx = (x + w * 0.5) / image_w
    cy = (y + h * 0.5) / image_h
    nw = w / image_w
    nh = h / image_h
    path.write_text(f'0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n', encoding='utf-8')


class DatasetCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('capture_paper_ball_dataset')
        self.args = args
        self.bridge = CvBridge()
        self.latest: np.ndarray | None = None
        self.create_subscription(Image, args.topic, self.image_callback, 1)

    def image_callback(self, msg: Image) -> None:
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image convert failed: {exc}')

    def wait_frame(self, timeout_sec: float = 6.0) -> np.ndarray:
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None:
                return self.latest.copy()
        raise TimeoutError(f'no image received from {self.args.topic}')


def main() -> None:
    parser = argparse.ArgumentParser(description='Capture low-view paper ball images and YOLO labels.')
    parser.add_argument('--topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--out-dir', default='/home/sunrise/trash_robot_v3/runtime/datasets/paper_ball')
    parser.add_argument('--count', type=int, default=80)
    parser.add_argument('--interval', type=float, default=0.18)
    parser.add_argument('--split', default='train', choices=('train', 'val'))
    parser.add_argument('--prefix', default='paper_ball')
    parser.add_argument('--min-area', type=float, default=250.0)
    parser.add_argument('--no-label', action='store_true')
    args = parser.parse_args()

    root = Path(args.out_dir)
    image_dir = root / 'images' / args.split
    label_dir = root / 'labels' / args.split
    preview_dir = root / 'preview' / args.split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)
    (root / 'paper_ball.yaml').write_text(
        f'path: {root}\ntrain: images/train\nval: images/val\n\nnames:\n  0: crumpled_paper\n',
        encoding='utf-8',
    )

    rclpy.init()
    node = DatasetCapture(args)
    saved = 0
    labeled = 0
    try:
        # Warm up subscription.
        node.wait_frame(timeout_sec=8.0)
        for idx in range(args.count):
            frame = node.wait_frame(timeout_sec=4.0)
            h, w = frame.shape[:2]
            stamp = time.strftime('%Y%m%d_%H%M%S')
            name = f'{args.prefix}_{args.split}_{stamp}_{idx:04d}'
            image_path = image_dir / f'{name}.jpg'
            label_path = label_dir / f'{name}.txt'
            preview_path = preview_dir / f'{name}.jpg'
            cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            saved += 1
            bbox = None if args.no_label else detect_white_paper_ball(frame, min_area=args.min_area)
            preview = frame.copy()
            if bbox is not None:
                write_yolo_label(label_path, bbox, w, h)
                x, y, bw, bh = bbox
                cv2.rectangle(preview, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(preview, 'crumpled_paper', (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                labeled += 1
            else:
                label_path.write_text('', encoding='utf-8')
                cv2.putText(preview, 'NO_AUTO_LABEL', (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            cv2.imwrite(str(preview_path), preview, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
            time.sleep(max(0.0, args.interval))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f'captured={saved} labeled={labeled} root={root}')
    print(f'yaml={root / "paper_ball.yaml"}')


if __name__ == '__main__':
    main()
