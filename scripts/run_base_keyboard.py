#!/usr/bin/env python3
"""Foreground keyboard teleop for the mobile base.

This is intentionally a small standalone tool under scripts/ because it is a
manual maintenance aid, not a production ROS node. It publishes only /cmd_vel
and sends several zero commands on exit.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


KEY_HELP = """\
键盘底盘遥控
  方向键 / W A S D / I J K L:
    ↑/W/I 前进     ↓/S/, 后退
    ←/A/J 左转     →/D/L 右转
    空格/K 停止
  速度档位:
    1 低速   2 中速   3 高速
  Q 或 Ctrl-C 退出，退出时自动发送零速度
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Trash Robot V3 base keyboard teleop")
    parser.add_argument("--topic", default="/cmd_vel", help="Twist topic to publish")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish rate")
    parser.add_argument("--deadman-sec", type=float, default=0.35, help="Stop if no key refresh")
    parser.add_argument("--probe", action="store_true", help="Initialize and publish zero only")
    return parser.parse_args()


def read_key(timeout_sec: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return None
    first = sys.stdin.read(1)
    if first != "\x1b":
        return first
    suffix = ""
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.001)
        if not ready:
            break
        suffix += sys.stdin.read(1)
        if len(suffix) >= 2:
            break
    return first + suffix


def zero_twist() -> Twist:
    return Twist()


def make_twist(linear_x: float, angular_z: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.angular.z = float(angular_z)
    return msg


def publish_zero(pub, count: int = 10, delay_sec: float = 0.04) -> None:
    msg = zero_twist()
    for _ in range(count):
        pub.publish(msg)
        time.sleep(delay_sec)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("trash_robot_v3_base_keyboard")
    pub = node.create_publisher(Twist, args.topic, 10)
    time.sleep(0.2)

    if args.probe:
        publish_zero(pub, count=3)
        node.destroy_node()
        rclpy.shutdown()
        print(f"OK: publisher ready on {args.topic}; zero command sent")
        return 0

    print(KEY_HELP)
    print(f"topic={args.topic} rate={args.rate_hz:.1f}Hz deadman={args.deadman_sec:.2f}s")
    print("安全提示：确认地面安全、急停可用；按 Q 退出。")

    speed_profiles = {
        "1": (0.06, 0.22),
        "2": (0.10, 0.35),
        "3": (0.16, 0.50),
    }
    linear_speed, angular_speed = speed_profiles["2"]
    current = zero_twist()
    last_command_time = 0.0
    period = 1.0 / max(args.rate_hz, 1.0)
    old_settings = termios.tcgetattr(sys.stdin)

    key_to_command = {
        "\x1b[A": (1.0, 0.0),
        "w": (1.0, 0.0),
        "W": (1.0, 0.0),
        "i": (1.0, 0.0),
        "I": (1.0, 0.0),
        "\x1b[B": (-1.0, 0.0),
        "s": (-1.0, 0.0),
        "S": (-1.0, 0.0),
        ",": (-1.0, 0.0),
        "\x1b[D": (0.0, 1.0),
        "a": (0.0, 1.0),
        "A": (0.0, 1.0),
        "j": (0.0, 1.0),
        "J": (0.0, 1.0),
        "\x1b[C": (0.0, -1.0),
        "d": (0.0, -1.0),
        "D": (0.0, -1.0),
        "l": (0.0, -1.0),
        "L": (0.0, -1.0),
    }

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            loop_start = time.monotonic()
            key = read_key(0.02)
            if key:
                if key in ("q", "Q", "\x03"):
                    break
                if key in speed_profiles:
                    linear_speed, angular_speed = speed_profiles[key]
                    print(f"\r速度档位 {key}: linear={linear_speed:.2f} angular={angular_speed:.2f}     ", end="")
                elif key in (" ", "k", "K"):
                    current = zero_twist()
                    last_command_time = 0.0
                elif key in key_to_command:
                    linear_dir, angular_dir = key_to_command[key]
                    current = make_twist(linear_dir * linear_speed, angular_dir * angular_speed)
                    last_command_time = time.monotonic()

            if last_command_time and time.monotonic() - last_command_time > args.deadman_sec:
                current = zero_twist()
                last_command_time = 0.0

            pub.publish(current)
            rclpy.spin_once(node, timeout_sec=0.0)
            elapsed = time.monotonic() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        publish_zero(pub)
        node.destroy_node()
        rclpy.shutdown()
        print("\n已退出，已发送零速度。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
