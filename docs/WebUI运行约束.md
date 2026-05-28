# WebUI 运行约束

当前 WebUI 已恢复为现场控制台，入口是：

```bash
./scripts/start_web_console.sh start
```

WebUI 地址：

```text
http://192.168.1.121:8095
```

## 边界

WebUI 只做控制面和状态面：

- 可以调用 Manager/Mission 服务；
- 可以展示 ROS 状态、视频、地图、日志；
- 可以发起受控的导航、巡逻、抓取请求；
- 不直接启动硬件驱动；
- 不直接 kill 机器人进程；
- 不绕过 Manager 直接控制机械臂或底盘。

## 必须经过的服务

| 功能 | WebUI 应调用 |
| --- | --- |
| 底盘启动/停止 | `/trash_manager/start_base`、`/trash_manager/stop_base` |
| 相机启动/停止 | `/trash_manager/start_camera`、`/trash_manager/stop_camera` |
| 机械臂启动/停止 | `/trash_manager/start_arm`、`/trash_manager/stop_arm` |
| 导航启动/停止 | `/trash_manager/start_navigation`、`/trash_manager/stop_navigation` |
| 抓取 dry/live | `/trash_manager/start_grasp_vlm_dry`、`/trash_manager/start_grasp_vlm_live` |
| 巡逻 | `/trash_mission/start_patrol`、`/trash_mission/stop_patrol` |
| 急停 | `/trash_robot_v3/manager/estop_trigger`、`/trash_robot_v3/manager/estop_reset` |

## 与 RViz 的关系

地图导航调试以真正 RViz2 为准：

```bash
./scripts/start_rviz_web.sh start
```

浏览器访问：

```text
http://192.168.1.121:6080/vnc.html?autoconnect=1&resize=remote&path=websockify
```

WebUI 可以嵌入或链接这个 RViz Web 页面，但不要再维护不准确的自制仿 RViz 地图作为主导航判断依据。

## 排障顺序

WebUI 某个按钮异常时，不要先改前端。按下面顺序查：

1. 对应模块脚本是否能单独启动；
2. Manager 服务是否存在；
3. WebUI 调用的 API 是否返回 pending/ready/failed；
4. ROS topic/service/action 是否真实在线；
5. 对应 `runtime/logs/<module>/` 日志。

模块原始启动方式见 `docs/模块启动说明书.md`。
