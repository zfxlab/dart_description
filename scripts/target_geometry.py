#!/usr/bin/env python3
"""Functions of module slide position derived from URDF, with CSV sampling."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import xacro


def default_urdf():
    source = Path(__file__).resolve().parents[1] / "urdf/dart_system.urdf.xacro"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("dart_description")) / "urdf/dart_system.urdf.xacro"


def transform(xyz, rpy):
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


def numbers(element, attribute, default):
    return [float(v) for v in (element.getAttribute(attribute) if element else default).split()]


class Geometry:
    def __init__(self, path=None):
        self.path = Path(path if path is not None else default_urdf()).resolve()
        document = xacro.process_file(str(self.path))
        self.joints = {}
        for joint in document.getElementsByTagName("joint"):

            def child(tag, joint=joint):
                nodes = joint.getElementsByTagName(tag)
                return nodes[0] if nodes else None

            limit = child("limit")
            self.joints[child("child").getAttribute("link")] = {
                "parent": child("parent").getAttribute("link"),
                "name": joint.getAttribute("name"),
                "type": joint.getAttribute("type"),
                "origin": transform(
                    numbers(child("origin"), "xyz", "0 0 0"),
                    numbers(child("origin"), "rpy", "0 0 0"),
                ),
                "axis": np.array(numbers(child("axis"), "xyz", "1 0 0")),
                "limits": [float(limit.getAttribute(a)) for a in ("lower", "upper")]
                if limit
                else None,
            }
        self.slide = self.joints["dart_detection_module_link"]
        if self.slide["type"] != "prismatic":
            raise ValueError("检测模块关节必须为 prismatic")
        self.lower, self.upper = self.slide["limits"]
        self.frame = np.linalg.inv(self.pose("dart_base_link", 0))
        if not all(math.isfinite(v) for v in (self.lower, self.upper)) or self.lower > self.upper:
            raise ValueError("无效的滑轨限位")
        self.coefficients = {}
        for name, link in (
            ("module", "dart_detection_module_link"),
            ("green", "green_light_link"),
            ("armor", "armor_link"),
        ):
            start = (self.frame @ self.pose(link, 0))[:2, 3]
            slope = (self.frame @ self.pose(link, 1))[:2, 3] - start
            self.coefficients[name] = (start, slope)

    def pose(self, link, slide, visited=None):
        visited = set() if visited is None else visited
        if link in visited:
            raise ValueError("URDF 关节树存在循环")
        visited.add(link)
        if link not in self.joints:
            if link != "field_link":
                raise ValueError(f"坐标系 {link} 未连接到 field_link")
            return np.eye(4)
        joint = self.joints[link]
        motion = np.eye(4)
        if joint["name"] == self.slide["name"]:
            motion[:3, 3] = joint["axis"] * slide
        return self.pose(joint["parent"], slide, visited) @ joint["origin"] @ motion

    def sample(self, slide):
        """Evaluate the geometry functions at a slide position in metres."""
        if not math.isfinite(slide) or not self.lower <= slide <= self.upper:
            raise ValueError("滑轨位置必须为 URDF 限位内的有限数值")
        result = {"slide_m": slide}
        for name, (start, slope) in self.coefficients.items():
            x, y = start + slope * slide
            result[name + "_x_m"] = float(x)
            result[name + "_y_m"] = float(y)
            if name == "module":
                continue
            distance = math.hypot(x, y)
            if distance < 1e-9:
                raise ValueError("目标与镖架水平位置重合, yaw 未定义")
            result[name + "_distance_m"] = distance
            result[name + "_yaw_rad"] = -math.atan2(y, x)
        result["yaw_delta_rad"] = (
            result["armor_yaw_rad"] - result["green_yaw_rad"] + math.pi
        ) % (2 * math.pi) - math.pi
        result["distance_delta_m"] = result["armor_distance_m"] - result["green_distance_m"]
        return result

    def functions(self):
        """Human-readable analytic functions in dart_base_link XY coordinates."""
        lines = [f"s in [{self.lower:.12g}, {self.upper:.12g}] m; reference: dart_base_link"]
        for name, (start, slope) in self.coefficients.items():
            for index, axis in enumerate(("x", "y")):
                lines.append(
                    f"{name}_{axis}(s) = {start[index]:.12g} + ({slope[index]:.12g}) * s [m]"
                )
            if name != "module":
                lines.append(f"{name}_distance(s) = hypot({name}_x(s), {name}_y(s)) [m]")
                lines.append(f"{name}_yaw(s) = -atan2({name}_y(s), {name}_x(s)) [rad]")
        lines.extend(
            [
                "distance_delta(s) = armor_distance(s) - green_distance(s) [m]",
                "yaw_delta(s) = wrap_to_pi(armor_yaw(s) - green_yaw(s)) [rad]",
            ]
        )
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="从 URDF 推导检测模块滑轨位置的几何函数并导出 CSV")
    parser.add_argument("--urdf", type=Path, help="URDF / xacro 文件, 默认使用 dart_description")
    parser.add_argument(
        "--samples", type=int, default=561, help="全滑轨行程采样点数, 包含两端, 默认 561"
    )
    parser.add_argument("--output", type=Path, default=Path("target_geometry.csv"))
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("--samples 必须至少为 2")
    try:
        geometry = Geometry(args.urdf)
        rows = [
            geometry.sample(float(s))
            for s in np.linspace(geometry.lower, geometry.upper, args.samples)
        ]
        with args.output.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    except (ValueError, KeyError, OSError, xacro.XacroException) as error:
        parser.exit(1, f"错误: {error}\n")
    print(geometry.functions())
    print(f"CSV: {args.output.resolve()} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
