# Trash Robot V3

室内垃圾分类机器人 ROS2 工程，包含底盘、导航、巡逻任务、视觉检测、VLM 抓取、机械臂分拣和 WebUI 控制台。

## 主要模块

- `scripts/`: 现场启动、停止、巡逻打点、抓取、导航、相机、YOLO 检测等脚本。
- `src/trash_robot_mission/`: 巡逻、目标锁定、靠近、VLM 精识别、抓取后恢复巡逻的任务状态机。
- `src/trash_robot_grasp/`: 手眼转换、抓取规划、RoArm 抓取和分类投放。
- `src/trash_robot_vision/`: VLM 识别、YOLO 候选目标、深度定位、MJPEG 视频流。
- `src/trash_robot_web/`: WebUI 后端和静态页面。
- `config/`: 导航、抓取、巡逻路线、VLM 服务商、硬件参数。
- `docs/`: 中文操作说明和调试文档。

## 现场启动入口

推荐先阅读：

- `docs/模块启动说明书.md`
- `docs/建图导航操作.md`
- `docs/抓取操作.md`
- `docs/WebUI运行约束.md`
- `docs/V3模块边界与运行规则.md`

常用入口：

```bash
source scripts/source_v3.sh
./scripts/trash_stack.sh status-all
./scripts/start_navigation.sh start
./scripts/start_grasp.sh live
./scripts/start_yolo_detector.sh start
./scripts/start_web_console.sh start
```

## GitHub 提交范围

本仓库提交源码、脚本、配置和文档。以下内容不会提交：

- `build/`, `install/`, `log/`, `runtime/`
- API 密钥和运行时 secret
- 大模型文件，如 `.bin`, `.gguf`, `.pt`, `.onnx`
- 运行截图、日志、rosbag、采集数据集

大模型和现场密钥需要在 RDK 本机按文档单独放置。
