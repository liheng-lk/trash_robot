#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import yaml


DEFAULT_ROUTE_FILE = (
    Path(os.environ.get("TRASH_ROBOT_ROOT", "/home/sunrise/trash_robot_v3"))
    / "config"
    / "mission"
    / "patrol_routes.yaml"
)


def normalize_route_mode(value: str) -> str:
    mode = value.strip().lower().replace("-", "_")
    if mode in ("closed", "loop", "closed_loop"):
        return "closed_loop"
    if mode in ("open", "open_loop"):
        return "open_loop"
    if mode in ("once", "single"):
        return "once"
    raise ValueError(f"unsupported route mode: {value}")


def read_route_file(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"active_route": "manual_route", "routes": {}, "mission": {}}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        data = {"active_route": "manual_route", "routes": {}, "mission": {}}
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
        backup_path = backup_dir / f"{path.name}.before_route_manager_{stamp}"
        shutil.copy2(path, backup_path)
    header = (
        "# Patrol routes are map-frame poses in meters/degrees.\n"
        "# mode: closed_loop = p1->...->pn->p1; open_loop = p1->...->pn->...->p1; once = stop at pn.\n"
        "# Use scripts/record_patrol_waypoint.sh after AMCL is aligned in RViz.\n"
    )
    payload = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(header + payload, encoding="utf-8")
    tmp_path.replace(path)
    return backup_path


def route_mode(route: Any) -> str:
    if isinstance(route, dict):
        raw = route.get("mode")
        if raw:
            try:
                return normalize_route_mode(str(raw))
            except ValueError:
                pass
        return "closed_loop" if bool(route.get("loop", True)) else "open_loop"
    return "closed_loop"


def route_waypoints(route: Any) -> list[Any]:
    if isinstance(route, dict):
        points = route.get("waypoints", [])
    else:
        points = route
    return points if isinstance(points, list) else []


def ensure_route(data: dict[str, Any], route_name: str, mode: str = "closed_loop") -> dict[str, Any]:
    routes = data.setdefault("routes", {})
    route = routes.get(route_name)
    if not isinstance(route, dict):
        route = {"mode": mode, "loop": mode == "closed_loop", "waypoints": []}
        routes[route_name] = route
    route.setdefault("waypoints", [])
    if "mode" not in route:
        route["mode"] = route_mode(route)
    route["loop"] = route["mode"] == "closed_loop"
    return route


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manage saved patrol routes.")
    parser.add_argument("--file", default=str(DEFAULT_ROUTE_FILE), help="patrol_routes.yaml path.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="List saved routes.")

    use = sub.add_parser("use", help="Set active route.")
    use.add_argument("route")

    mode = sub.add_parser("mode", help="Set a route mode.")
    mode.add_argument("route")
    mode.add_argument("mode", choices=["closed_loop", "open_loop", "once", "closed", "open", "loop"])

    clear = sub.add_parser("clear", help="Clear waypoints in a route, keeping the route.")
    clear.add_argument("route")
    clear.add_argument("--mode", choices=["closed_loop", "open_loop", "once", "closed", "open", "loop"])
    clear.add_argument("--no-activate", action="store_true")

    delete = sub.add_parser("delete", help="Delete a route.")
    delete.add_argument("route")

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    route_file = Path(args.file).expanduser().resolve()
    data = read_route_file(route_file)
    routes = data.setdefault("routes", {})
    active = str(data.get("active_route") or "")

    if args.cmd == "list":
        print(f"active_route={active}")
        for name, route in routes.items():
            points = route_waypoints(route)
            names = [str(item.get("name", f"wp_{idx}")) for idx, item in enumerate(points) if isinstance(item, dict)]
            mark = "*" if name == active else " "
            print(f"{mark} {name}: mode={route_mode(route)} points={len(points)} names={','.join(names)}")
        return 0

    if args.cmd == "use":
        if args.route not in routes:
            print(f"ERROR: route not found: {args.route}", file=sys.stderr)
            return 2
        data["active_route"] = args.route
        backup = write_route_file(route_file, data)
        print(f"active_route={args.route}")
        if backup:
            print(f"backup={backup}")
        print('reload: ros2 service call /trash_mission/reload_route std_srvs/srv/Trigger "{}"')
        return 0

    if args.cmd == "mode":
        try:
            mode = normalize_route_mode(args.mode)
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        route = ensure_route(data, args.route, mode)
        route["mode"] = mode
        route["loop"] = mode == "closed_loop"
        backup = write_route_file(route_file, data)
        print(f"route={args.route} mode={mode}")
        if backup:
            print(f"backup={backup}")
        return 0

    if args.cmd == "clear":
        mode = normalize_route_mode(args.mode) if args.mode else "closed_loop"
        route = ensure_route(data, args.route, mode)
        route["mode"] = mode
        route["loop"] = mode == "closed_loop"
        route["waypoints"] = []
        if not args.no_activate:
            data["active_route"] = args.route
        backup = write_route_file(route_file, data)
        print(f"cleared route={args.route} mode={mode}")
        if backup:
            print(f"backup={backup}")
        return 0

    if args.cmd == "delete":
        if args.route not in routes:
            print(f"ERROR: route not found: {args.route}", file=sys.stderr)
            return 2
        del routes[args.route]
        if data.get("active_route") == args.route:
            data["active_route"] = next(iter(routes), "")
        backup = write_route_file(route_file, data)
        print(f"deleted route={args.route}")
        if backup:
            print(f"backup={backup}")
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
