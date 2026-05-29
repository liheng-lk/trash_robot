# Experimental Tools

这里放当前不作为现场默认链路的实验脚本。

## start_local_vlm.sh

本地 VLM 测试脚本，用于验证 RDK 官方 `hobot_llamacpp` 链路。它不控制机械臂，不替代当前 VLM API 抓取主链路。

常用命令：

```bash
tools/experimental/start_local_vlm.sh status
tools/experimental/start_local_vlm.sh once
tools/experimental/start_local_vlm.sh start
tools/experimental/start_local_vlm.sh stop
```

注意：本地 VLM 需要模型文件和足够 ION 内存，速度也未必满足巡逻实时性。
