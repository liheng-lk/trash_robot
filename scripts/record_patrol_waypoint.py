#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import yaml


DEFAULT_ROUTE_FILE = (
    Path(os.environ.get("TRASH_ROBOT_ROOT", "/home/sunrise/trash_robot_v3"))
    / "config"
    / "mission"
    / "patrol_routes.yaml"
)


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


def normalize_route_mode(value: str | None, no_loop: bool = False) -> str | None:
    if no_loop:
        return "open_loop"
    if value is None:
        return None
    mode = value.strip().lower().replace("-", "_")
    if mode in ("closed", "loop", "closed_loop"):
        return "closed_loop"
    if mode in ("open", "open_loop"):
        return "open_loop"
    if mode in ("once", "single"):
        return "once"
    raise ValueError(f"unsupported route mode: {value}")


class AmclPoseCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("record_patrol_waypoint")
        self.pose_msg: PoseWithCovarianceStamped | None = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseWithCovarianceStamped, topic, self.pose_callback, qos)

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose_msg = msg


def read_route_file(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"active_route": "manual_route", "routes": {}, "mission": {}}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return {"active_route": "manual_route", "routes": {}, "mission": {}}
    data.setdefault("routes", {})
    if not isinstance(data["routes"], dict):
        data["routes"] = {}
    data.setdefault("mission", {})
    if not isinstance(data["mission"], dict):
        data["mission"] = {}
    return data


def write_route_file(path: Path, data: dict[str, Any]) -> Path | None:
    backup_path: Path | None = None
    if path.exists():
        backup_dir = path.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup_path = backup_dir / f"{path.name}.before_record_{stamp}"
        shutil.copy2(path, backup_path)

    header = (
        "# Patrol routes are map-frame poses in meters/degrees.\n"
        "# Use scripts/record_patrol_waypoint.sh after AMCL is aligned in RViz.\n"
    )
    payload = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(header + payload, encoding="utf-8")
    tmp_path.replace(path)
    return backup_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record the current /amcl_pose as a patrol waypoint in patrol_routes.yaml."
    )
    parser.add_argument("name", help="Waypoint name, for example p1 or door_corner.")
    parser.add_argument("--route", help="Route name. Defaults to active_route in the YAML.")
    parser.add_argument("--file", default=str(DEFAULT_ROUTE_FILE), help="patrol_routes.yaml path.")
    parser.add_argument("--pose-topic", default="/amcl_pose", help="Pose topic to read.")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for AMCL pose.")
    parser.add_argument("--map-frame", default="map", help="Map frame used by TF fallback.")
    parser.add_argument("--base-frame", default="base_link", help="Robot base frame used by TF fallback.")
    parser.add_argument("--no-tf-fallback", action="store_true", help="Disable map->base_link TF fallback.")
    parser.add_argument("--dwell-sec", type=float, default=None, help="Optional dwell time metadata.")
    parser.add_argument(
        "--mode",
        choices=["closed_loop", "open_loop", "closed", "open", "loop", "once"],
        help="Route mode: closed_loop uses pn->p1; open_loop returns pn->...->p1; once stops at pn.",
    )
    parser.add_argument("--closed-loop", action="store_true", help="Set route mode to closed_loop.")
    parser.add_argument("--open-loop", action="store_true", help="Set route mode to open_loop.")
    parser.add_argument("--reset", action="store_true", help="Clear the selected route before saving.")
    parser.add_argument("--no-loop", action="store_true", help="Compatibility alias for --open-loop.")
    parser.add_argument("--no-activate", action="store_true", help="Do not set active_route to this route.")
    parser.add_argument(
        "--append-duplicate",
        action="store_true",
        help="Append even if a waypoint with the same name already exists.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    route_file = Path(args.file).expanduser().resolve()
    data = read_route_file(route_file)
    route_name = args.route or str(data.get("active_route") or "manual_route")
    mode_arg = args.mode
    if args.closed_loop:
        mode_arg = "closed_loop"
    if args.open_loop:
        mode_arg = "open_loop"
    try:
        requested_mode = normalize_route_mode(mode_arg, args.no_loop)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = AmclPoseCapture(args.pose_topic)
    deadline = time.monotonic() + max(args.timeout, 0.1)
    source = args.pose_topic
    try:
        while rclpy.ok() and node.pose_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose_msg is not None:
            pose = node.pose_msg.pose.pose
            x = round(float(pose.position.x), 3)
            y = round(float(pose.position.y), 3)
            yaw_deg = round(normalize_angle_deg(math.degrees(yaw_from_quaternion(pose.orientation))), 1)
        elif not args.no_tf_fallback:
            tf_deadline = time.monotonic() + 2.0
            transform = None
            while rclpy.ok() and transform is None and time.monotonic() < tf_deadline:
                try:
                    transform = node.tf_buffer.lookup_transform(args.map_frame, args.base_frame, Time())
                except Exception:
                    rclpy.spin_once(node, timeout_sec=0.1)
            if transform is None:
                print(
                    f"ERROR: no pose received from {args.pose_topic} within {args.timeout:.1f}s, "
                    f"and no TF {args.map_frame}->{args.base_frame} was available. "
                    "Check AMCL/map and RViz 2D Pose Estimate.",
                    file=sys.stderr,
                )
                return 2
            x = round(float(transform.transform.translation.x), 3)
            y = round(float(transform.transform.translation.y), 3)
            yaw_deg = round(
                normalize_angle_deg(math.degrees(yaw_from_quaternion(transform.transform.rotation))),
                1,
            )
            source = f"tf:{args.map_frame}->{args.base_frame}"
        else:
            print(
                f"ERROR: no pose received from {args.pose_topic} within {args.timeout:.1f}s. "
                "Check AMCL/map and RViz 2D Pose Estimate.",
                file=sys.stderr,
            )
            return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()

    routes = data.setdefault("routes", {})
    if args.reset or route_name not in routes or not isinstance(routes.get(route_name), dict):
        mode = requested_mode or "closed_loop"
        routes[route_name] = {"mode": mode, "loop": mode == "closed_loop", "waypoints": []}
    route = routes[route_name]
    if requested_mode is not None:
        route["mode"] = requested_mode
        route["loop"] = requested_mode == "closed_loop"
    elif "mode" not in route:
        route["mode"] = "closed_loop" if bool(route.get("loop", True)) else "open_loop"
    route["loop"] = str(route.get("mode", "closed_loop")).strip().lower() in ("closed", "loop", "closed_loop")
    waypoints = route.setdefault("waypoints", [])
    if not isinstance(waypoints, list):
        waypoints = []
        route["waypoints"] = waypoints

    waypoint: dict[str, Any] = {"name": args.name, "x": x, "y": y, "yaw_deg": yaw_deg}
    if args.dwell_sec is not None:
        waypoint["dwell_sec"] = round(float(args.dwell_sec), 2)

    replaced = False
    if not args.append_duplicate:
        for index, item in enumerate(waypoints):
            if isinstance(item, dict) and item.get("name") == args.name:
                waypoints[index] = waypoint
                replaced = True
                break
    if not replaced:
        waypoints.append(waypoint)

    if not args.no_activate:
        data["active_route"] = route_name

    backup_path = write_route_file(route_file, data)
    action = "updated" if replaced else "added"
    print(f"{action} waypoint route={route_name} name={args.name} x={x:.3f} y={y:.3f} yaw_deg={yaw_deg:.1f}")
    print(f"mode={route.get('mode')}")
    print(f"source={source}")
    print(f"route_file={route_file}")
    if backup_path:
        print(f"backup={backup_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
