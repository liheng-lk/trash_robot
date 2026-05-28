# 多模态 API 垃圾分类

V3 当前阶段采用 VLM API 精准分类链路。本地 BPU/COCO 模型后续再作为可替换后端接入，不参与当前默认运行。VLM API 只负责识别物品、分类、给出抓取点和框；深度、手眼变换、安全窗口、抓取动作仍由本地 ROS 节点完成。

## 启动

默认使用 VLM API。先设置阿里云百炼密钥：

```bash
export DASHSCOPE_API_KEY=你的阿里云百炼Key
./scripts/start_grasp.sh dry
```

可选指定 provider：

```bash
./scripts/start_grasp.sh dry vlm dashscope
```

## 输出 topic

VLM 节点发布兼容现有抓取链路的 topic：

```text
/trash_target_pixel
/trash_target_label
/trash_target_raw_label
/trash_detection_status
```

并新增调试 topic：

```text
/trash_vlm_result
/trash_target_bbox
```

## 分类规则

```text
水果、果皮、剩饭、食物残渣 -> GARBAGE_KITCHEN
纸张、纸团、纸盒、塑料瓶、易拉罐、玻璃瓶 -> GARBAGE_RECYCLE
电池、充电宝、电子小件、药品 -> GARBAGE_HAZARD
无法明确但确实是垃圾 -> GARBAGE_OTHER
非垃圾、人体、桌椅、背景 -> 不发布抓取目标
```

## 安全策略

- API Key 只从环境变量读取，不写入配置文件、不写入日志。
- 默认限频 0.5Hz，超时 8 秒。
- JSON 解析失败、类别非法、bbox 越界、置信度低都会 fail-closed，不发布抓取目标。
- 第一阶段只选当前画面最适合抓取的一个目标。

## 测试

```bash
python3 -m compileall -q src/trash_robot_vision
pytest -q tests/test_vlm_trash_classifier.py
```

RDK 上：

```bash
cd /home/sunrise/trash_robot_v3
colcon build
export DASHSCOPE_API_KEY=...
./scripts/start_grasp.sh dry

ros2 topic echo /trash_vlm_result --once
ros2 topic echo /trash_target_pixel --once
ros2 topic echo /trash_target_label --once
ros2 topic echo /trash_target_point_arm --once
```

手眼偏置实测在 VLM 分类链路稳定后再做。
