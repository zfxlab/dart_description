#!/usr/bin/env python3
"""Draw the configured base ROI and the dart base reference ground."""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class EnvironmentVisualizer(Node):
    def __init__(self):
        super().__init__("environment_visualizer")
        defaults = {
            "base_roi.enabled": True,
            "base_roi.frame_id": "base_link",
            "base_roi.lower_m": [-0.68, -0.40, 0.0],
            "base_roi.upper_m": [0.60, 0.50, 0.90],
            "ground.enabled": True,
            "ground.frame_id": "field_link",
            "ground.z_m": 0.205,
            "ground.x_bounds_m": [0.0, 15.0],
            "ground.y_bounds_m": [0.0, 28.0],
            "ground.spacing_m": 1.0,
            "ground.line_width_m": 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for prefix in ("base_roi",):
            lower = self.get_parameter(f"{prefix}.lower_m").value
            upper = self.get_parameter(f"{prefix}.upper_m").value
            if (len(lower) != 3 or len(upper) != 3
                    or any(not math.isfinite(v) for v in (*lower, *upper))
                    or any(a >= b for a, b in zip(lower, upper, strict=True))):
                raise ValueError(f"{prefix} bounds must be finite three-element vectors with lower < upper")
            if not self.get_parameter(f"{prefix}.frame_id").value:
                raise ValueError(f"{prefix}.frame_id must not be empty")
        for name in ("ground.x_bounds_m", "ground.y_bounds_m"):
            bounds = self.get_parameter(name).value
            if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]:
                raise ValueError(f"{name} must contain two increasing finite values")
        for name in ("ground.spacing_m", "ground.line_width_m"):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.get_parameter("ground.z_m").value):
            raise ValueError("ground.z_m must be finite")
        if not self.get_parameter("ground.frame_id").value:
            raise ValueError("ground.frame_id must not be empty")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/environment_markers", qos)
        self.markers = self.make_markers()
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()
        self.get_logger().info(
            f"Publishing base ROI and ground z={self.get_parameter('ground.z_m').value:.3f} m"
        )

    @staticmethod
    def marker(namespace, marker_id, kind, frame, color):
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns, marker.id, marker.type = namespace, marker_id, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    @staticmethod
    def point(x, y, z):
        return Point(x=float(x), y=float(y), z=float(z))

    def roi_markers(self, namespace, color):
        frame = self.get_parameter(f"{namespace}.frame_id").value
        lower = self.get_parameter(f"{namespace}.lower_m").value
        upper = self.get_parameter(f"{namespace}.upper_m").value
        box = self.marker(namespace, 0, Marker.CUBE, frame, (*color, 0.15))
        center = [(a + b) / 2 for a, b in zip(lower, upper, strict=True)]
        box.pose.position = self.point(*center)
        box.scale.x, box.scale.y, box.scale.z = [b - a for a, b in zip(lower, upper, strict=True)]
        edges = self.marker(namespace, 1, Marker.LINE_LIST, frame, (*color, 1.0))
        edges.scale.x = 0.005
        corners = [self.point(*[upper[axis] if index & (1 << axis) else lower[axis] for axis in range(3)]) for index in range(8)]
        for index in range(8):
            for axis in range(3):
                other = index ^ (1 << axis)
                if index < other:
                    edges.points.extend((corners[index], corners[other]))
        return [box, edges]

    def make_markers(self):
        markers = []
        if self.get_parameter("base_roi.enabled").value:
            markers.extend(self.roi_markers("base_roi", (0.15, 0.45, 1.0)))
        if self.get_parameter("ground.enabled").value:
            grid = self.marker("ground", 0, Marker.LINE_LIST, self.get_parameter("ground.frame_id").value, (0.25, 0.35, 0.25, 0.8))
            grid.scale.x = self.get_parameter("ground.line_width_m").value
            xmin, xmax = self.get_parameter("ground.x_bounds_m").value
            ymin, ymax = self.get_parameter("ground.y_bounds_m").value
            z = self.get_parameter("ground.z_m").value
            spacing = self.get_parameter("ground.spacing_m").value
            for low, high, along_x in ((xmin, xmax, True), (ymin, ymax, False)):
                count = math.floor(high / spacing) - math.ceil(low / spacing) + 1
                if count > 10000:
                    raise ValueError("Ground spacing produces too many grid lines")
                coordinates = {low, high}
                coordinates.update((math.ceil(low / spacing) + i) * spacing for i in range(count))
                for value in sorted(coordinates):
                    endpoints = ((value, ymin, z), (value, ymax, z)) if along_x else ((xmin, value, z), (xmax, value, z))
                    grid.points.extend(self.point(*p) for p in endpoints)
            markers.append(grid)
        return MarkerArray(markers=markers)

    def publish(self):
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EnvironmentVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
