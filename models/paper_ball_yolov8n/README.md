# Paper Ball YOLOv8n Model

This package contains the first custom paper-ball detector trained from robot-camera images.

## Files

- `paper_ball_yolov8n_best.pt`: Ultralytics YOLOv8n training checkpoint.
- `paper_ball_yolov8n_ultralytics.onnx`: standard Ultralytics ONNX export. This is useful for desktop validation, but it is not the TROS `dnn_node_example` parser format.
- `paper_ball_yolov8n_bpu6_op11.onnx`: six-output ONNX shaped for the D-Robotics/TROS YOLOv8 parser.
- `crumpled_paper.list`: one-class label file.
- `yolov8_paper_ball_workconfig.json`: runtime config expected after conversion to X5 `.bin`.

## Current Status

The RDK runtime does not include `hb_mapper`, so the `.onnx` still needs to be converted in an OpenExplorer/D-Robotics model conversion environment before it can run with `dnn_node_example`.

Expected converted file:

```text
/home/sunrise/trash_robot_v3/models/paper_ball_yolov8n/paper_ball_yolov8n_640x640_nv12.bin
```

Once that file exists on the robot, start the detector with:

```bash
TRASH_YOLO_PROFILE=paper_ball ./scripts/start_yolo_detector.sh restart
```

The default profile remains the official COCO YOLO model:

```bash
./scripts/start_yolo_detector.sh restart
```

## Conversion

The local conversion helper follows the official RDK Model Zoo YOLO mapping flow:

```bash
tools/run_paper_ball_x5_conversion.sh
```

For config/calibration preparation only:

```bash
python3 tools/convert_paper_ball_yolo_x5.py --prepare-only --samples 20
```

On Apple Silicon, the OpenExplorer Docker image must run as `linux/amd64`.
