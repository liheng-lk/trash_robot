#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class MapToMarker(Node):
    def __init__(self):
        super().__init__("map_to_marker")
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.max_points = int(self.declare_parameter("max_points", 2500).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 50).value)
        self.marker = None
        self.pub = self.create_publisher(Marker, "/map_occupied_marker", marker_qos)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_timer(2.0, self.publish)
        self.get_logger().info("Waiting for /map and publishing lightweight /map_occupied_marker")

    def on_map(self, msg: OccupancyGrid):
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        yaw = self._yaw_from_quaternion(msg.info.origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        boundary = []
        data = msg.data
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                i = row + x
                if data[i] < self.occupied_threshold:
                    continue
                neighbors = (data[i - 1], data[i + 1], data[i - width], data[i + width])
                if all(n >= self.occupied_threshold for n in neighbors):
                    continue
                boundary.append((x, y))

        if len(boundary) > self.max_points:
            step = max(1, math.ceil(len(boundary) / self.max_points))
            boundary = boundary[::step][: self.max_points]

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or "map"
        marker.ns = "map_outline"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.06, resolution * 1.2)
        marker.scale.y = max(0.06, resolution * 1.2)
        marker.color = ColorRGBA(r=0.92, g=0.92, b=0.88, a=0.95)

        points = []
        for x, y in boundary:
            cx = (x + 0.5) * resolution
            cy = (y + 0.5) * resolution
            points.append(
                Point(
                    x=origin_x + cx * cos_yaw - cy * sin_yaw,
                    y=origin_y + cx * sin_yaw + cy * cos_yaw,
                    z=0.02,
                )
            )
        marker.points = points
        self.marker = marker
        self.get_logger().info(f"Converted map {width}x{height}: outline_points={len(points)}")
        self.publish()

    def publish(self):
        if self.marker is None:
            return
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = MapToMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
