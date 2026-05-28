#!/usr/bin/env python3
"""Read-only analysis of latest robot snapshot. Does not control hardware."""
from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SNAP_ROOT = ROOT / "runtime" / "snapshots"
REPORT_PATH = ROOT / "runtime" / "reports" / "last_review.md"

CRITICAL_TOPICS = [
    "/cmd_vel",
    "/odom",
    "/scan",
    "/camera/camera/color/image_raw",
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/color/camera_info",
    "/trash_grasp_plan",
    "/trash_target_camera_point",
    "/trash_target_point_camera",
    "/trash_target_point_arm",
    "/trash_target_arm_point",
]

OPTIONAL_TOPICS = [
    "/arm/status",
    "/trash_robot/state",
    "/trash_perception_status",
]


def find_latest_snapshot() -> Path | None:
    latest = SNAP_ROOT / "latest"
    if latest.is_symlink() or latest.exists():
        try:
            target = latest.resolve()
            if target.is_dir():
                return target
        except OSError:
            pass
    if not SNAP_ROOT.is_dir():
        return None
    dirs = sorted([p for p in SNAP_ROOT.iterdir() if p.is_dir() and p.name != "latest"], reverse=True)
    return dirs[0] if dirs else None


def read_text(path: Path, limit: int = 200_000) -> str:
    if not path.is_file():
        return ""
    try:
        return path.read_text(encoding="utf-8", errors="replace")[:limit]
    except OSError:
        return ""


def topic_listed(topic: str, topic_list_blob: str) -> bool:
    return topic in topic_list_blob or topic.rstrip("/") in topic_list_blob


def parse_hz_from_outputs(snap: Path, topic: str) -> tuple[str, float | None]:
    safe = topic.replace("/", "_")
    candidates = [
        snap / "topics" / f"hz{safe}.out",
        snap / "commands" / f"hz{safe}.out",
    ]
    for path in candidates:
        body = read_text(path)
        if not body:
            continue
        m = re.search(r"average rate:\s*([\d.]+)", body)
        if m:
            return "ok", float(m.group(1))
        low = body.lower()
        if "does not appear to be published" in low or "unknown topic" in low:
            return "missing", None
        if "no messages" in low or "could not determine" in low:
            return "no_messages", None
    return "unknown", None


def analyze_depth_grasp_plan(snap: Path) -> list[str]:
    issues: list[str] = []
    blob = ""
    for path in list(snap.glob("topics/echo_trash_grasp_plan.out")) + list(
        snap.glob("commands/echo_trash_grasp_plan.out")
    ):
        blob += read_text(path)
    if not blob.strip():
        issues.append("No /trash_grasp_plan echo captured (perception stack likely down).")
        return issues
    try:
        # ros2 echo may print multiple lines; find JSON-ish payload
        m = re.search(r"\{.*\}", blob, re.DOTALL)
        if m:
            data = json.loads(m.group(0))
            if not data.get("depth_ok", False):
                issues.append(
                    f"grasp_plan depth_ok=false reason={data.get('depth_reason', data.get('reason', '?'))}"
                )
        elif "depth_ok" in blob and "false" in blob.lower():
            issues.append("grasp_plan suggests depth_ok=false")
    except json.JSONDecodeError:
        if "depth_ok" not in blob:
            issues.append("grasp_plan missing depth_ok field (contract not met).")
    return issues


def main() -> int:
    snap = find_latest_snapshot()
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)

    problems: list[tuple[int, str]] = []  # priority, message
    lines: list[str] = []

    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    lines.append("# Last Snapshot Review")
    lines.append("")
    lines.append(f"- **Generated:** {now}")
    lines.append(f"- **Reviewer:** `review_last_snapshot.py` (read-only)")
    lines.append("")

    if snap is None:
        lines.append("## Status: FAIL")
        lines.append("")
        lines.append("No snapshot found under `runtime/snapshots/`.")
        lines.append("")
        lines.append("```bash")
        lines.append("./scripts/diagnostics/collect_robot_snapshot.sh")
        lines.append("```")
        problems.append((1, "No snapshot directory"))
        REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"Wrote {REPORT_PATH}")
        return 1

    lines.append(f"- **Snapshot:** `{snap.relative_to(ROOT)}`")
    lines.append("")

    topic_list = read_text(snap / "commands" / "ros2_topic_list.out")
    if not topic_list:
        # fallback: any topic list file
        for p in snap.glob("commands/*topic_list*"):
            topic_list = read_text(p)
            break

    ros2_ok = (snap / "commands" / "ros2_node_list.out").exists() or "ros2 CLI not available" not in read_text(
        snap / "commands/ros2_missing.out"
    )

    if "ros2 CLI not available" in read_text(snap / "commands/ros2_missing.out"):
        problems.append((2, "ros2 CLI not available in snapshot environment"))
        lines.append("## ROS environment: WARN (ros2 not in PATH)")
    else:
        lines.append("## ROS environment: OK (ros2 invoked)")
    lines.append("")

    lines.append("## Topic presence")
    lines.append("")
    lines.append("| Topic | Listed | Hz status |")
    lines.append("|-------|--------|-----------|")

    for topic in CRITICAL_TOPICS + OPTIONAL_TOPICS:
        listed = topic_listed(topic, topic_list) if topic_list else False
        hz_status, hz_val = parse_hz_from_outputs(snap, topic)
        hz_note = hz_status if hz_val is None else f"{hz_status} ({hz_val:.2f} Hz)"
        lines.append(f"| `{topic}` | {'yes' if listed else '**no**'} | {hz_note} |")
        if topic in CRITICAL_TOPICS and not listed and topic_list:
            problems.append((3, f"Critical topic missing from graph: {topic}"))

    lines.append("")
    lines.append("## Chain checks")
    lines.append("")

    color_ok = topic_listed("/camera/camera/color/image_raw", topic_list)
    depth_ok_topic = topic_listed("/camera/camera/aligned_depth_to_color/image_raw", topic_list)
    if color_ok and not depth_ok_topic:
        problems.append((2, "Color image present but aligned depth topic missing"))
        lines.append("- **Depth chain:** FAIL — aligned depth not advertised")
    elif depth_ok_topic:
        lines.append("- **Depth chain:** PASS (aligned depth topic exists)")
    else:
        lines.append("- **Depth chain:** WARN (camera stack not running)")
        problems.append((3, "Camera stack not running"))

    cam_point = topic_listed("/trash_target_camera_point", topic_list) or topic_listed(
        "/trash_target_point_camera", topic_list
    )
    arm_point = topic_listed("/trash_target_point_arm", topic_list) or topic_listed(
        "/trash_target_arm_point", topic_list
    )
    if cam_point and arm_point:
        lines.append("- **Hand-eye chain:** topics exist for camera + arm points")
    elif cam_point and not arm_point:
        problems.append((2, "Camera point topic without arm point (handeye/transform down?)"))
        lines.append("- **Hand-eye chain:** WARN — arm point missing")
    else:
        lines.append("- **Hand-eye chain:** WARN — perception not publishing points")

    lines.append("")
    for issue in analyze_depth_grasp_plan(snap):
        problems.append((2, issue))
        lines.append(f"- **Grasp plan:** {issue}")

    arm_status = topic_listed("/arm/status", topic_list)
    if not arm_status:
        lines.append("- **Arm safety:** WARN — `/arm/status` not present (Phase 4 wrapper pending)")
    else:
        lines.append("- **Arm safety:** `/arm/status` advertised")

    lines.append("")
    lines.append("## Top 3 likely issues")
    lines.append("")
    problems.sort(key=lambda x: x[0])
    top3 = problems[:3]
    if not top3:
        lines.append("1. No critical issues detected (or ROS was offline during snapshot).")
        lines.append("2. Run snapshot while stack is up for meaningful hz/echo data.")
        lines.append("3. Proceed to Phase 1 PROJECT_AUDIT if static review is next.")
        next_action = "Execute Phase 1: generate `docs/architecture/PROJECT_AUDIT.md`."
    else:
        for i, (_, msg) in enumerate(top3, 1):
            lines.append(f"{i}. {msg}")
        next_action = f"Address highest priority: {top3[0][1]}"

    lines.append("")
    lines.append("## Next single action")
    lines.append("")
    lines.append(next_action)
    lines.append("")
    lines.append("---")
    lines.append("*This script does not modify code or control hardware.*")

    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {REPORT_PATH}")
    return 0 if not top3 or top3[0][0] >= 3 else 0


if __name__ == "__main__":
    raise SystemExit(main())
