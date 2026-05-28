# V3 hbm_runtime BPU 推理接入规则

## 当前状态

当前阶段主线仍为 VLM API。本文件定义后续在 RDK X5 上接入本地 BPU 模型时必须遵守的硬规则。未完成 hbm 模型和 wrapper 前，BPU 后端必须保持关闭。

## 强制规则

1. 只允许使用 `hbm_runtime` 接口，禁止使用旧 `hobot_dnn` 接口。
2. 输入图像必须转换为 NV12 packed 格式，尺寸必须严格匹配模型输入。
3. 全局只允许加载一个模型实例，禁止频繁加载和卸载。
4. 推理完成后必须立即释放输入/输出 tensor 内存；长时间不用时主动卸载模型。
5. 所有推理操作必须有异常处理，错误必须 fail-closed，不能导致节点崩溃后继续运动。
6. ROS2 图像接入使用 `cv_bridge`，并尽量避免额外内存复制。
7. BPU 同一时间只能执行一个推理任务，必须通过 `BPU_INFERENCE` 锁串行化。
8. 所有推理错误必须记录详细日志，包括模型名、输入尺寸、格式、错误码、耗时、BPU busy 状态。
9. 预留本地模型名为 `qwen3.5:2b`；只有转换为 RDK X5 可运行 hbm/INT8 模型并通过验证后才允许启用。

## 输出契约

本地 BPU 后端必须输出与当前 VLM API 主线等价的结果字段：

```text
object_class
grasp_point
confidence
```

底层推理节点不得直接控制底盘、机械臂、相机或 WebUI。

## 配置文件

配置集中在：

```text
config/vision/bpu_runtime_policy.yaml
```

默认 `enabled: false`。启用前必须补齐模型文件、输入宽高、格式和 wrapper。

## 禁止事项

- 禁止使用 `hobot_dnn`。
- 禁止 CPU 大模型推理。
- 禁止 GPU/CUDA/TensorRT/OpenVINO 推理。
- 禁止同时加载多个模型。
- 禁止 WebUI 直接调用模型。
- 禁止推理失败后继续进入抓取状态机。
