#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def write_label(label_path: Path, bbox: tuple[int, int, int, int], image_w: int, image_h: int) -> None:
    x, y, w, h = bbox
    cx = (x + w * 0.5) / image_w
    cy = (y + h * 0.5) / image_h
    nw = w / image_w
    nh = h / image_h
    label_path.write_text(f'0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(description='Apply one fixed crumpled_paper bbox to a captured group.')
    parser.add_argument('--root', default='/home/sunrise/trash_robot_v3/runtime/datasets/paper_ball')
    parser.add_argument('--split', default='train', choices=('train', 'val'))
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--bbox', required=True, help='x,y,w,h in pixels')
    args = parser.parse_args()

    root = Path(args.root)
    image_dir = root / 'images' / args.split
    label_dir = root / 'labels' / args.split
    preview_dir = root / 'preview' / args.split
    label_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)
    bbox = tuple(int(float(v)) for v in args.bbox.split(','))
    if len(bbox) != 4:
        raise SystemExit('--bbox must be x,y,w,h')

    count = 0
    for image_path in sorted(image_dir.glob(f'{args.prefix}_*.jpg')):
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        x, y, bw, bh = bbox
        x = max(0, min(w - 1, x))
        y = max(0, min(h - 1, y))
        bw = max(1, min(w - x, bw))
        bh = max(1, min(h - y, bh))
        fixed = (x, y, bw, bh)
        label_path = label_dir / f'{image_path.stem}.txt'
        write_label(label_path, fixed, w, h)
        preview = img.copy()
        cv2.rectangle(preview, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.putText(preview, 'crumpled_paper', (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imwrite(str(preview_dir / image_path.name), preview, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        count += 1
    print(f'applied={count} prefix={args.prefix} bbox={bbox} split={args.split}')


if __name__ == '__main__':
    main()
