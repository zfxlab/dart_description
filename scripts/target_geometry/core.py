"""Callable geometry derived from the dart system URDF/xacro model."""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import xacro


SLIDE_JOINT = "dart_detection_module_slide_joint"
YAW_JOINT = "launcher_yaw_joint"


@dataclass(frozen=True)
class Surface:
    """A finite rectangular surface expressed in a requested reference frame."""

    center: np.ndarray
    normal: np.ndarray
    axis_u: np.ndarray
    axis_v: np.ndarray
    size_u: float
    size_v: float

    def coordinates(self, point):
        """Return surface u, v and signed normal distance for a reference-frame point."""
        offset = np.asarray(point, dtype=float) - self.center
        return np.array(
            [
                np.dot(offset, self.axis_u),
                np.dot(offset, self.axis_v),
                np.dot(offset, self.normal),
            ]
        )

    def contains(self, point, normal_tolerance=1e-9):
        """Return whether a point lies on the finite surface within the given tolerance."""
        u, v, normal = self.coordinates(point)
        return bool(
            abs(u) <= self.size_u / 2
            and abs(v) <= self.size_v / 2
            and abs(normal) <= normal_tolerance
        )


def default_urdf():
    source = Path(__file__).resolve().parents[2] / "urdf/dart_system.urdf.xacro"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("dart_description")) / "urdf/dart_system.urdf.xacro"


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    """Construct a homogeneous transform from URDF xyz and fixed-axis RPY values."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = xyz
    return result


def axis_rotation(axis, angle):
    """Construct a homogeneous rotation around an arbitrary joint axis."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("关节旋转轴必须为有限非零向量")
    x, y, z = axis / norm
    c, s, one_minus_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4)
    result[:3, :3] = [
        [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
    ]
    return result


def _numbers(element, attribute, default):
    text = element.getAttribute(attribute) if element else default
    return [float(value) for value in text.split()]


class Geometry:
    """Evaluate link poses and target geometry without duplicating URDF constants."""

    def __init__(self, path=None):
        self.path = Path(path if path is not None else default_urdf()).resolve()
        self.document = xacro.process_file(str(self.path))
        self.joints = {}
        for joint in self.document.getElementsByTagName("joint"):

            def child(tag, joint=joint):
                nodes = joint.getElementsByTagName(tag)
                return nodes[0] if nodes else None

            child_element = child("child")
            parent_element = child("parent")
            if child_element is None or parent_element is None:
                raise ValueError(f"关节 {joint.getAttribute('name')} 缺少 parent 或 child")
            limit = child("limit")
            self.joints[child_element.getAttribute("link")] = {
                "parent": parent_element.getAttribute("link"),
                "name": joint.getAttribute("name"),
                "type": joint.getAttribute("type"),
                "origin": transform(
                    _numbers(child("origin"), "xyz", "0 0 0"),
                    _numbers(child("origin"), "rpy", "0 0 0"),
                ),
                "axis": np.array(_numbers(child("axis"), "xyz", "1 0 0")),
                "limits": tuple(float(limit.getAttribute(name)) for name in ("lower", "upper"))
                if limit and limit.hasAttribute("lower") and limit.hasAttribute("upper")
                else None,
            }

        self._joints_by_name = {joint["name"]: joint for joint in self.joints.values()}
        self._require_joint(SLIDE_JOINT, "prismatic")
        self._require_joint(YAW_JOINT, "revolute")
        self.lower, self.upper = self.joint_limits(SLIDE_JOINT)
        if not all(math.isfinite(value) for value in (self.lower, self.upper)):
            raise ValueError("滑轨限位必须为有限数值")
        if self.lower > self.upper:
            raise ValueError("滑轨下限不能大于上限")
        self.armor_size = self._box_size("armor_link")

    def _require_joint(self, name, expected_type):
        joint = self._joints_by_name.get(name)
        if joint is None:
            raise ValueError(f"URDF 缺少关节 {name}")
        if joint["type"] != expected_type:
            raise ValueError(f"关节 {name} 必须为 {expected_type}")

    def _box_size(self, link_name):
        for link in self.document.getElementsByTagName("link"):
            if link.getAttribute("name") != link_name:
                continue
            visuals = link.getElementsByTagName("visual")
            if not visuals:
                break
            boxes = visuals[0].getElementsByTagName("box")
            if boxes:
                size = np.array(_numbers(boxes[0], "size", ""))
                if size.shape == (3,) and np.all(np.isfinite(size)) and np.all(size > 0.0):
                    return size
            break
        raise ValueError(f"坐标系 {link_name} 必须包含有效的 box visual")

    def joint_limits(self, joint_name):
        joint = self._joints_by_name.get(joint_name)
        if joint is None or joint["limits"] is None:
            raise ValueError(f"关节 {joint_name} 没有有限位")
        return joint["limits"]

    @property
    def slide_limits(self):
        return self.lower, self.upper

    def _joint_motion(self, joint, position):
        if not math.isfinite(position):
            raise ValueError(f"关节 {joint['name']} 位置必须为有限数值")
        joint_type = joint["type"]
        if joint_type == "fixed":
            if position != 0.0:
                raise ValueError(f"固定关节 {joint['name']} 的位置必须为零")
            return np.eye(4)
        if joint_type == "prismatic":
            result = np.eye(4)
            result[:3, 3] = joint["axis"] * position
            return result
        if joint_type in ("revolute", "continuous"):
            return axis_rotation(joint["axis"], position)
        raise ValueError(f"不支持的关节类型: {joint_type}")

    def pose(self, link, joint_positions=None, reference="field_link"):
        """Return the pose of ``link`` in ``reference`` for named joint positions."""
        positions = {} if joint_positions is None else dict(joint_positions)
        world_link = self._world_pose(link, positions, set())
        world_reference = self._world_pose(reference, positions, set())
        return np.linalg.inv(world_reference) @ world_link

    def _world_pose(self, link, positions, visited):
        if link in visited:
            raise ValueError("URDF 关节树存在循环")
        visited.add(link)
        if link not in self.joints:
            if link != "field_link":
                raise ValueError(f"坐标系 {link} 未连接到 field_link")
            return np.eye(4)
        joint = self.joints[link]
        position = float(positions.get(joint["name"], 0.0))
        motion = self._joint_motion(joint, position)
        return self._world_pose(joint["parent"], positions, visited) @ joint["origin"] @ motion

    def _positions(self, slide=0.0, launcher_yaw=0.0):
        if not math.isfinite(slide) or not self.lower <= slide <= self.upper:
            raise ValueError("滑轨位置必须为 URDF 限位内的有限数值")
        yaw_lower, yaw_upper = self.joint_limits(YAW_JOINT)
        if not math.isfinite(launcher_yaw) or not yaw_lower <= launcher_yaw <= yaw_upper:
            raise ValueError("launcher yaw 必须为 URDF 限位内的有限数值")
        return {SLIDE_JOINT: slide, YAW_JOINT: launcher_yaw}

    def link_pose(self, link, slide=0.0, launcher_yaw=0.0, reference="dart_base_link"):
        """Return a commonly used link pose with validated slide and yaw positions."""
        return self.pose(link, self._positions(slide, launcher_yaw), reference)

    def position(self, link, slide=0.0, launcher_yaw=0.0, reference="dart_base_link"):
        """Return a link origin in the requested reference frame."""
        return self.link_pose(link, slide, launcher_yaw, reference)[:3, 3].copy()

    def module_position(self, slide, reference="dart_base_link"):
        return self.position("dart_detection_module_link", slide, reference=reference)

    def green_position(self, slide, reference="dart_base_link"):
        return self.position("green_light_link", slide, reference=reference)

    def armor_position(self, slide, reference="dart_base_link"):
        """Return the center of the armor upper surface."""
        return self.position("armor_link", slide, reference=reference)

    def armor_surface(self, slide, reference="dart_base_link"):
        """Return the armor upper surface: local y/z span it and local +x is normal."""
        pose = self.link_pose("armor_link", slide, reference=reference)
        rotation = pose[:3, :3]
        return Surface(
            center=pose[:3, 3].copy(),
            normal=rotation[:, 0].copy(),
            axis_u=rotation[:, 1].copy(),
            axis_v=rotation[:, 2].copy(),
            size_u=float(self.armor_size[1]),
            size_v=float(self.armor_size[0]),
        )

    def launcher_origin(self, reference="dart_base_link"):
        return self.position("launcher_frame", reference=reference)

    def release_pose(self, launcher_yaw, reference="dart_base_link"):
        return self.link_pose("dart_release_link", launcher_yaw=launcher_yaw, reference=reference)

    def release_position(self, launcher_yaw, reference="dart_base_link"):
        return self.release_pose(launcher_yaw, reference)[:3, 3].copy()

    def launch_direction(self, launcher_yaw, reference="dart_base_link"):
        """Return the unit initial direction given by dart_release_link local +x."""
        return self.release_pose(launcher_yaw, reference)[:3, 0].copy()

    def target_yaw(self, slide, link="armor_link", reference="dart_base_link"):
        """Return the horizontal bearing from the launcher yaw axis to a target link."""
        target = self.position(link, slide, reference=reference)
        launcher = self.launcher_origin(reference)
        delta = target - launcher
        distance = math.hypot(delta[0], delta[1])
        if distance <= 1e-12:
            raise ValueError("目标与 launcher yaw 轴的水平位置重合，yaw 未定义")
        return math.atan2(delta[1], delta[0])

    def horizontal_distance(self, slide, link="armor_link", reference="dart_base_link"):
        target = self.position(link, slide, reference=reference)
        launcher = self.launcher_origin(reference)
        return float(np.linalg.norm((target - launcher)[:2]))

    def surface_coordinates(self, point, slide, reference="dart_base_link"):
        return self.armor_surface(slide, reference).coordinates(point)
