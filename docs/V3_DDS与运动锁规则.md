# V3 DDS 与运动锁规则

## 目标

V3 所有本项目脚本统一使用 CycloneDDS 与同一个 ROS 域，避免 WebUI、Manager、抓取节点和调试命令处在不同 DDS 环境里，造成“服务在线但调用超时”的假象。

## DDS 规则

- 默认 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- 默认 `ROS_DOMAIN_ID=1`
- 默认 `CYCLONEDDS_URI=file://$TRASH_ROBOT_ROOT/config/dds/cyclonedds_unicast.xml`
- 禁止脚本自动启用 FastDDS SHM。
- `cyclonedds_unicast.xml` 不强制设置 `SocketReceiveBufferSize min=10MB`，避免 RDK 当前内核 buffer 上限导致 ROS2 节点创建失败。
- 如果需要测试 TROS/FastDDS 专用链路，必须作为独立实验工具处理，不能污染主运行脚本。

## 运动锁

文件：

- `/tmp/trash_robot_v3_motion.lock`
- `/tmp/trash_robot_v3_estop.lock`

规则：

- 同一时间只允许一个运动 owner 持有 motion lock。
- `ESTOP` 持有锁时，任何新的运动请求必须失败。
- `reset_estop` 只解除软急停锁，不代表可以直接进入 live，仍需 Safety Gate 和人工确认。

## 软急停入口

命令：

```bash
./scripts/start_estop.sh trigger manual
./scripts/start_estop.sh status
./scripts/start_estop.sh reset
```

触发软急停时会写入锁文件，并尝试向 `/cmd_vel` 和 `/trash_robot_v3/base/cmd_vel` 各发布 10 次零速度。该脚本不启动底盘、不启动机械臂、不做真实运动。

## Manager 状态闭环

Manager 会把同一份状态发布到：

- `/trash_system_status`：旧 WebUI/调试兼容入口
- `/trash_robot_v3/manager/system_state`：V3 标准状态入口

状态字段包含：

- `dds`：当前 RMW、ROS domain、CycloneDDS 配置路径
- `motion_lock`：运动锁 owner、软急停状态、锁文件路径
- `system_state`：`current_state`、`fault_code`、`can_start_navigation`、`can_start_grasp` 等门控状态

WebUI 急停按钮现在调用 Manager 的 `/trash_robot_v3/manager/estop_trigger`，解除按钮调用 `/trash_robot_v3/manager/estop_reset`。页面上的“模式锁 / 运动锁 / DDS”来自 Manager 状态，不再只依赖浏览器本地变量。

## 禁止事项

- 禁止 `pkill -f ros2`
- 禁止 `pkill -f python`
- 禁止 `pkill -f trash_robot`
- 禁止 WebUI 绕过 Manager 直接控制硬件
- 禁止在急停锁存在时继续抓取、导航或手动控制机械臂
