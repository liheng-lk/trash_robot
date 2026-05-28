#!/usr/bin/env python3
"""Convert the custom paper-ball YOLOv8n ONNX into an RDK X5 BPU .bin model.

Run this inside the D-Robotics OpenExplorer X5 toolchain environment where
`hb_mapper` is available.
"""

from __future__ import annotations

import argparse
import os
import random
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--onnx",
        default="models/paper_ball_yolov8n/paper_ball_yolov8n_bpu6_op11.onnx",
        help="BPU/TROS-compatible six-output ONNX path.",
    )
    parser.add_argument(
        "--cal-images",
        default="paper_ball_dataset/images/train",
        help="Calibration image directory.",
    )
    parser.add_argument(
        "--output-dir",
        default="models/paper_ball_yolov8n",
        help="Directory to receive the final .bin and conversion logs.",
    )
    parser.add_argument(
        "--output-prefix",
        default="paper_ball_yolov8n_640x640_nv12",
        help="Output .bin filename prefix.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--optimize-level", default="O3")
    parser.add_argument("--workspace", default="models/paper_ball_yolov8n/x5_mapper_workspace")
    parser.add_argument("--prepare-only", action="store_true", help="Only write calibration data and config.yaml.")
    return parser.parse_args()


def collect_images(cal_dir: Path, sample_count: int) -> list[Path]:
    images = sorted(
        p
        for p in cal_dir.iterdir()
        if p.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )
    if not images:
        raise SystemExit(f"no calibration images found in {cal_dir}")
    if len(images) > sample_count:
        random.Random(20260528).shuffle(images)
        images = sorted(images[:sample_count])
    return images


def write_calibration_data(images: list[Path], dst: Path, width: int, height: int) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True, exist_ok=True)

    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"WARN: skip unreadable image {image_path}")
            continue
        tensor = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = cv2.resize(tensor, (width, height))
        tensor = np.transpose(tensor, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0).astype(np.float32)
        tensor.tofile(dst / f"{image_path.name}.rgbchw")


def write_mapper_yaml(
    config_path: Path,
    onnx_path: Path,
    output_prefix: str,
    working_dir: Path,
    cal_data_dir: Path,
    jobs: int,
    optimize_level: str,
) -> None:
    config_path.write_text(
        f"""model_parameters:
  onnx_model: '{onnx_path}'
  march: "bayes-e"
  layer_out_dump: False
  working_dir: '{working_dir}'
  output_model_file_prefix: '{output_prefix}'
input_parameters:
  input_name: ""
  input_type_rt: 'nv12'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  norm_type: 'data_scale'
  scale_value: 0.003921568627451
calibration_parameters:
  cal_data_dir: '{cal_data_dir}'
  cal_data_type: 'float32'
  calibration_type: 'default'
  optimization: set_Softmax_input_int8,set_Softmax_output_int8
compiler_parameters:
  jobs: {jobs}
  compile_mode: 'latency'
  debug: true
  optimize_level: '{optimize_level}'
""",
        encoding="utf-8",
    )


def run(command: list[str], cwd: Path) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def main() -> None:
    args = parse_args()
    repo = Path.cwd()
    onnx_path = (repo / args.onnx).resolve()
    cal_dir = (repo / args.cal_images).resolve()
    output_dir = (repo / args.output_dir).resolve()
    workspace = (repo / args.workspace).resolve()
    cal_data_dir = workspace / "calibration_data_rgb_f32"
    bpu_output_dir = workspace / "bpu_model_output"
    config_path = workspace / "config.yaml"

    if not onnx_path.exists():
        raise SystemExit(f"ONNX missing: {onnx_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    workspace.mkdir(parents=True, exist_ok=True)

    images = collect_images(cal_dir, args.samples)
    print(f"calibration images: {len(images)} from {cal_dir}")
    write_calibration_data(images, cal_data_dir, args.width, args.height)
    write_mapper_yaml(
        config_path=config_path,
        onnx_path=onnx_path,
        output_prefix=args.output_prefix,
        working_dir=bpu_output_dir,
        cal_data_dir=cal_data_dir,
        jobs=args.jobs,
        optimize_level=args.optimize_level,
    )

    if args.prepare_only:
        print(f"mapper config: {config_path}")
        print(f"calibration data: {cal_data_dir}")
        return

    run(["hb_mapper", "checker", "--model-type", "onnx", "--config", str(config_path)], repo)
    run(["hb_mapper", "makertbin", "--model-type", "onnx", "--config", str(config_path)], repo)

    built_bin = bpu_output_dir / f"{args.output_prefix}.bin"
    if not built_bin.exists():
        raise SystemExit(f"converted .bin not found: {built_bin}")

    final_bin = output_dir / f"{args.output_prefix}.bin"
    shutil.copy2(built_bin, final_bin)
    for log_name in ("hb_mapper_checker.log", "hb_mapper_makertbin.log"):
        log_path = repo / log_name
        if log_path.exists():
            shutil.copy2(log_path, output_dir / log_name)
    shutil.copy2(config_path, output_dir / "x5_mapper_config.yaml")
    print(f"converted model: {final_bin}")


if __name__ == "__main__":
    main()
