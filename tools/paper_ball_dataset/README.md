# Paper Ball Dataset Tools

这些工具用于纸团 YOLO 数据采集和标注修正，不属于现场机器人启动入口。

## 采集数据

```bash
source scripts/source_v3.sh
python3 tools/paper_ball_dataset/capture_paper_ball_dataset.py --count 80 --split train
```

默认输出到：

```text
runtime/datasets/paper_ball
```

该目录被 `.gitignore` 忽略，不会提交到 GitHub。

## 批量套用一个框

```bash
python3 tools/paper_ball_dataset/apply_paper_ball_bbox.py \
  --prefix paper_ball_train_20260528_120000 \
  --bbox 120,180,90,70
```

`--bbox` 使用图像像素坐标 `x,y,w,h`。
