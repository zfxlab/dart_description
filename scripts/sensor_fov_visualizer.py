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

        width = self.get_parameter("camera.width").value
        height = self.get_parameter("camera.height").value
        fx = self.get_parameter("camera.fx").value
        fy = self.get_parameter("camera.fy").value
        cx = self.get_parameter("camera.cx").value
        cy = self.get_parameter("camera.cy").value
        horizontal_fov = math.degrees(
            math.atan(cx / fx) + math.atan((width - cx) / fx)
        )
        vertical_fov = math.degrees(
            math.atan(cy / fy) + math.atan((height - cy) / fy)
        )
        self.get_logger().info(
            "Camera FOV %.2f x %.2f deg; lidar shape=%s, range=%.2f..%.2f m"
            % (
                horizontal_fov,
                vertical_fov,
                self.get_parameter("lidar.shape").value,
                self.get_parameter("lidar.min_range_m").value,
                self.get_parameter("lidar.max_range_m").value,
            )
        )

    def _declare_parameters(self):
        defaults = {
            "camera.enabled": True,
            "camera.frame_id": "camera_optical_frame",
            "camera.width": 1440,
            "camera.height": 1080,
            "camera.fx": 1200.0,
            "camera.fy": 1200.0,
            "camera.cx": 720.0,
            "camera.cy": 540.0,
            "camera.near_m": 0.1,
            "camera.far_m": 10.0,
            "camera.line_color": [0.1, 0.85, 1.0, 1.0],
            "lidar.enabled": True,
            "lidar.frame_id": "livox_frame",
            "lidar.shape": "circular",
            "lidar.circular_fov_deg": 70.4,
            "lidar.horizontal_fov_deg": 70.4,
            "lidar.vertical_fov_deg": 70.4,
            "lidar.min_range_m": 0.1,
            "lidar.max_range_m": 15.0,
            "lidar.ring_segments": 64,
            "lidar.line_color": [1.0, 0.75, 0.05, 1.0],
            "marker.scale": [0.03, 0.15, 0.24],
            "publish_period_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _validate_parameters(self):
        positive = (
            "camera.width",
            "camera.height",
            "camera.fx",
            "camera.fy",
            "camera.near_m",
            "camera.far_m",
            "lidar.min_range_m",
            "lidar.max_range_m",
            "publish_period_s",
        )
        for name in positive:
            if self.get_parameter(name).value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.get_parameter("camera.far_m").value <= self.get_parameter(
            "camera.near_m"
        ).value:
            raise ValueError("camera.far_m must be greater than camera.near_m")
        if self.get_parameter("lidar.max_range_m").value <= self.get_parameter(
            "lidar.min_range_m"
        ).value:
            raise ValueError("lidar.max_range_m must be greater than lidar.min_range_m")
        shape = self.get_parameter("lidar.shape").value
        if shape not in ("circular", "rectangular"):
            raise ValueError("lidar.shape must be circular or rectangular")
        if self.get_parameter("lidar.ring_segments").value < 8:
            raise ValueError("lidar.ring_segments must be at least 8")
        width = self.get_parameter("camera.width").value
        height = self.get_parameter("camera.height").value
        cx = self.get_parameter("camera.cx").value
        cy = self.get_parameter("camera.cy").value
        if not 0.0 <= cx <= width or not 0.0 <= cy <= height:
            raise ValueError("camera principal point must be inside the image")
        for name in (
            "lidar.circular_fov_deg",
            "lidar.horizontal_fov_deg",
            "lidar.vertical_fov_deg",
        ):
            angle = self.get_parameter(name).value
            if angle <= 0.0 or angle >= 180.0:
                raise ValueError(f"{name} must be in (0, 180) degrees")
        for name in (
            "camera.line_color",
            "lidar.line_color",
        ):
            color = self.get_parameter(name).value
            if len(color) != 4 or any(value < 0.0 or value > 1.0 for value in color):
                raise ValueError(f"{name} must contain four values in [0, 1]")
        marker_scale = self.get_parameter("marker.scale").value
        if len(marker_scale) != 3 or any(value <= 0.0 for value in marker_scale):
            raise ValueError("marker.scale must contain three positive values")

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

    def _camera_corners(self, distance):
        width = self.get_parameter("camera.width").value
        height = self.get_parameter("camera.height").value
        fx = self.get_parameter("camera.fx").value
        fy = self.get_parameter("camera.fy").value
        cx = self.get_parameter("camera.cx").value
        cy = self.get_parameter("camera.cy").value
        return [
            point((u - cx) * distance / fx, (v - cy) * distance / fy, distance)
            for u, v in ((0, 0), (width, 0), (width, height), (0, height))
        ]

    def _camera_markers(self):
        frame = self.get_parameter("camera.frame_id").value
        near = self._camera_corners(self.get_parameter("camera.near_m").value)
        far = self._camera_corners(self.get_parameter("camera.far_m").value)
        line = self._marker(
            "camera",
            0,
            Marker.LINE_LIST,
            frame,
            self.get_parameter("camera.line_color").value,
        )
        for index in range(4):
            add_segment(line.points, near[index], near[(index + 1) % 4])
            add_segment(line.points, far[index], far[(index + 1) % 4])
            add_segment(line.points, near[index], far[index])

        direction = self._direction_marker(
            "camera",
            1,
            frame,
            self.get_parameter("camera.line_color").value,
            point(0.0, 0.0, self.get_parameter("camera.far_m").value),
        )
        return [line, direction]

    @staticmethod
    def _rectangular_lidar_corner(distance, horizontal_angle, vertical_angle):
        direction = (1.0, math.tan(horizontal_angle), math.tan(vertical_angle))
        norm = math.sqrt(sum(component * component for component in direction))
        return point(*(distance * component / norm for component in direction))

    def _rectangular_lidar_rings(self):
        horizontal = math.radians(
            self.get_parameter("lidar.horizontal_fov_deg").value / 2.0
        )
        vertical = math.radians(
            self.get_parameter("lidar.vertical_fov_deg").value / 2.0
        )
        angles = (
            (-horizontal, vertical),
            (horizontal, vertical),
            (horizontal, -vertical),
            (-horizontal, -vertical),
        )
        near_distance = self.get_parameter("lidar.min_range_m").value
        far_distance = self.get_parameter("lidar.max_range_m").value
        near = [
            self._rectangular_lidar_corner(near_distance, h_angle, v_angle)
            for h_angle, v_angle in angles
        ]
        far = [
            self._rectangular_lidar_corner(far_distance, h_angle, v_angle)
            for h_angle, v_angle in angles
        ]
        return near, far

    def _circular_lidar_rings(self):
        segments = self.get_parameter("lidar.ring_segments").value
        half_angle = math.radians(
            self.get_parameter("lidar.circular_fov_deg").value / 2.0
        )

        def ring(distance):
            axial = distance * math.cos(half_angle)
            radius = distance * math.sin(half_angle)
            return [
                point(
                    axial,
                    radius * math.cos(2.0 * math.pi * index / segments),
                    radius * math.sin(2.0 * math.pi * index / segments),
                )
                for index in range(segments)
            ]

        return (
            ring(self.get_parameter("lidar.min_range_m").value),
            ring(self.get_parameter("lidar.max_range_m").value),
        )

    def _lidar_markers(self):
        frame = self.get_parameter("lidar.frame_id").value
        if self.get_parameter("lidar.shape").value == "circular":
            near, far = self._circular_lidar_rings()
        else:
            near, far = self._rectangular_lidar_rings()

        line = self._marker(
            "lidar",
            0,
            Marker.LINE_LIST,
            frame,
            self.get_parameter("lidar.line_color").value,
        )
        connector_step = max(1, len(near) // 8)
        for index in range(len(near)):
            next_index = (index + 1) % len(near)
            add_segment(line.points, near[index], near[next_index])
            add_segment(line.points, far[index], far[next_index])
            if index % connector_step == 0:
                add_segment(line.points, near[index], far[index])

        direction = self._direction_marker(
            "lidar",
            1,
            frame,
            self.get_parameter("lidar.line_color").value,
            point(self.get_parameter("lidar.max_range_m").value, 0.0, 0.0),
        )
        return [line, direction]

    def _publish_markers(self):
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        if self.get_parameter("camera.enabled").value:
            markers.extend(self._camera_markers())
        if self.get_parameter("lidar.enabled").value:
            markers.extend(self._lidar_markers())
        message = MarkerArray()
        message.markers = markers
        self._publisher.publish(message)
        subscription_count = self._publisher.get_subscription_count()
        if subscription_count != self._last_subscription_count:
            visible_count = sum(
                marker.action == Marker.ADD for marker in message.markers
            )
            self.get_logger().info(
                "Published %d visible markers; RViz/topic subscribers=%d"
                % (visible_count, subscription_count)
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
