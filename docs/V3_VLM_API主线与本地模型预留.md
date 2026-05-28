# V3 VLM API 主线与本地模型预留

## 当前阶段结论

当前阶段识别主线采用 VLM API。后续如果需要接入 RDK X5 本地 BPU/INT8 模型，必须作为独立后端接入，不能和 VLM API 同时运行。

## 运行边界

- VLM API 只负责输出物品类别、抓取点、置信度和调试信息。
- 深度定位、手眼转换、安全窗口、机械臂动作必须由本地 ROS 节点完成。
- WebUI 只能通过 Manager 启停 VLM dry/live，不能直接调用模型或打开硬件。
- API key 只允许来自环境变量或运行期私有密钥文件，禁止写入源码、配置和日志。

## 本地模型预留规则

- 本地模型后续放入 `models/` 或受控部署目录。
- 本地模型必须有单模型锁，禁止多模型同时加载。
- 本地模型后端必须输出与 VLM API 相同的 ROS 结果接口。
- 本地模型接入前，`bpu`、`coco`、`mono2d`、`trash4` 参数必须 fail-closed。
- RDK X5 本地模型后端只能使用 `hbm_runtime`，禁止旧 `hobot_dnn`。
- 图像输入必须是 NV12 packed，尺寸严格匹配模型输入。
- 预留本地模型名为 `qwen3.5:2b`，只有完成 hbm/INT8 转换和实机验证后才允许启用。

## 禁止事项

- 禁止 CPU 大模型推理替代 VLM API。
- 禁止 GPU、CUDA、TensorRT、OpenVINO 路径进入 RDK 主运行链路。
- 禁止 WebUI 直接调用模型。
- 禁止抓取节点直接打开模型文件或相机设备。
- 禁止为了测试绕过深度、TF、安全窗口。

## 阶段 1 验收

```bash
python3 -m compileall -q src/trash_robot_*
bash -n scripts/*.sh
bash -n scripts/lib/*.sh
grep -RniE "cuda|tensorrt|openvino|gpu|model_training|trash4_|trash_label_map" scripts src config docs tests 2>/dev/null || true
./scripts/start_grasp.sh dry bpu
```

`./scripts/start_grasp.sh dry bpu` 应明确失败，提示当前阶段只启用 VLM API，本地模型后续接入。
