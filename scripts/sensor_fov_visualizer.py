#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def point(x, y, z):
    result = Point()
    result.x = float(x)
    result.y = float(y)
    result.z = float(z)
    return result


def add_segment(points, start, end):
    points.extend((start, end))


class SensorFovVisualizer(Node):
    def __init__(self):
        super().__init__("sensor_fov_visualizer")
        self._declare_parameters()
        self._validate_parameters()

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._publisher = self.create_publisher(
            MarkerArray, "/sensor_fov_markers", qos
        )
        self._last_subscription_count = -1
        period = self.get_parameter("publish_period_s").value
        self._timer = self.create_timer(period, self._publish_markers)
        self._publish_markers()

        for camera in ("left_camera", "right_camera"):
            width, height, fx, fy, cx, cy = [
                self._value(camera, name) for name in ("width", "height", "fx", "fy", "cx", "cy")
            ]
            horizontal = math.degrees(math.atan(cx / fx) + math.atan((width - cx) / fx))
            vertical = math.degrees(math.atan(cy / fy) + math.atan((height - cy) / fy))
            self.get_logger().info(f"{camera}: FOV {horizontal:.2f} x {vertical:.2f} deg")

    def _value(self, camera, name):
        return self.get_parameter(f"{camera}.{name}").value

    def _declare_parameters(self):
        for camera, color in (
            ("left_camera", [0.1, 0.85, 1.0, 1.0]),
            ("right_camera", [1.0, 0.55, 0.15, 1.0]),
        ):
            defaults = {
                "enabled": True,
                "frame_id": f"{camera}_optical_frame",
                "width": 1440,
                "height": 1080,
                "fx": 9955.98186,
                "fy": 9940.26158,
                "cx": 704.45187,
                "cy": 569.55719,
                "near_m": 0.01,
                "far_m": 26.0,
                "line_color": color,
                "surface_alpha": 0.08,
            }
            for name, value in defaults.items():
                self.declare_parameter(f"{camera}.{name}", value)
        self.declare_parameter("marker.scale", [0.005, 0.03, 0.08])
        self.declare_parameter("publish_period_s", 1.0)

    def _validate_parameters(self):
        for camera in ("left_camera", "right_camera"):
            for name in ("width", "height", "fx", "fy", "near_m", "far_m"):
                value = self._value(camera, name)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{camera}.{name} must be finite and positive")
            if self._value(camera, "far_m") <= self._value(camera, "near_m"):
                raise ValueError(f"{camera}.far_m must exceed near_m")
            cx, cy = self._value(camera, "cx"), self._value(camera, "cy")
            width, height = self._value(camera, "width"), self._value(camera, "height")
            if not all(math.isfinite(v) for v in (cx, cy)):
                raise ValueError(f"{camera} principal point must be finite")
            if not 0 <= cx <= width or not 0 <= cy <= height:
                raise ValueError(f"{camera} principal point must be inside the image")
            if not self._value(camera, "frame_id"):
                raise ValueError(f"{camera}.frame_id must not be empty")
            color = self._value(camera, "line_color")
            if len(color) != 4 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in color):
                raise ValueError(f"{camera}.line_color must contain four values in [0, 1]")
            alpha = self._value(camera, "surface_alpha")
            if not math.isfinite(alpha) or not 0 <= alpha <= 1:
                raise ValueError(f"{camera}.surface_alpha must be in [0, 1]")
        scale = self.get_parameter("marker.scale").value
        if len(scale) != 3 or any(not math.isfinite(v) or v <= 0 for v in scale):
            raise ValueError("marker.scale must contain three finite positive values")
        period = self.get_parameter("publish_period_s").value
        if not math.isfinite(period) or period <= 0:
            raise ValueError("publish_period_s must be finite and positive")

    def _marker(self, namespace, marker_id, marker_type, frame_id, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        # A zero stamp asks RViz for the latest transform.  Using "now" here can
        # put the marker slightly ahead of robot_state_publisher's newest TF.
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker_scale = self.get_parameter("marker.scale").value
        marker.scale.x = marker_scale[0]
        marker.scale.y = marker_scale[1]
        marker.scale.z = marker_scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.frame_locked = True
        return marker

    def _direction_marker(self, namespace, marker_id, frame_id, color, end):
        marker = self._marker(namespace, marker_id, Marker.ARROW, frame_id, color)
        marker.points = [point(0.0, 0.0, 0.0), end]
        return marker

    def _camera_corners(self, camera, distance):
        width, height, fx, fy, cx, cy = [
            self._value(camera, name) for name in ("width", "height", "fx", "fy", "cx", "cy")
        ]
        # Ideal pinhole image boundary in optical coordinates (z forward, x right, y down).
        return [
            point((u - cx) * distance / fx, (v - cy) * distance / fy, distance)
            for u, v in ((0, 0), (width, 0), (width, height), (0, height))
        ]

    def _camera_markers(self, camera):
        frame = self._value(camera, "frame_id")
        color = self._value(camera, "line_color")
        near = self._camera_corners(camera, self._value(camera, "near_m"))
        far = self._camera_corners(camera, self._value(camera, "far_m"))
        line = self._marker(camera, 0, Marker.LINE_LIST, frame, color)
        for index in range(4):
            add_segment(line.points, near[index], near[(index + 1) % 4])
            add_segment(line.points, far[index], far[(index + 1) % 4])
            add_segment(line.points, near[index], far[index])
        direction = self._direction_marker(
            camera, 1, frame, color, point(0.0, 0.0, self._value(camera, "far_m"))
        )
        surface = self._marker(camera, 2, Marker.TRIANGLE_LIST, frame, color)
        surface.scale.x = surface.scale.y = surface.scale.z = 1.0
        surface.color.a = self._value(camera, "surface_alpha")
        for index in range(4):
            nxt = (index + 1) % 4
            surface.points.extend((near[index], far[index], far[nxt], near[index], far[nxt], near[nxt]))
        for ring in (near, far):
            surface.points.extend((ring[0], ring[1], ring[2], ring[0], ring[2], ring[3]))
        return [line, direction, surface]

    def _publish_markers(self):
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        for camera in ("left_camera", "right_camera"):
            if self._value(camera, "enabled"):
                markers.extend(self._camera_markers(camera))
        message = MarkerArray()
        message.markers = markers
        self._publisher.publish(message)
        subscription_count = self._publisher.get_subscription_count()
        if subscription_count != self._last_subscription_count:
            visible_count = sum(
                marker.action == Marker.ADD for marker in message.markers
            )
            self.get_logger().info(
                f"Published {visible_count} visible markers; "
                f"RViz/topic subscribers={subscription_count}"
            )
            self._last_subscription_count = subscription_count


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorFovVisualizer()
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
