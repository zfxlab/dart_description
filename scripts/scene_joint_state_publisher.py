#!/usr/bin/env python3
"""Publish configurable visualization poses for field mechanisms, not launcher yaw."""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import JointState


class SceneJointStatePublisher(Node):
    def __init__(self):
        super().__init__('scene_joint_state_publisher')
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('robot_description', '', readonly)
        root = ET.fromstring(self.get_parameter('robot_description').value)
        self.ranges = {}
        for side in ('red', 'blue'):
            for parameter, joint in [('base_slide_m', 'base_target_slide_joint'),
                                     ('gate_open_m', 'dart_gate_joint')]:
                limit = root.find(f"joint[@name='{side}_{joint}']/limit")
                lo, hi = float(limit.get('lower')), float(limit.get('upper'))
                if not math.isfinite(lo) or not math.isfinite(hi) or not lo <= 0 <= hi:
                    raise ValueError(f'Invalid limits for {side}_{joint}')
                self.ranges[f'{side}_{parameter}'] = (lo, hi)
        for name in self.ranges:
            self.declare_parameter(name, 0.0)
        check = self.validate([self.get_parameter(n) for n in self.ranges])
        if not check.successful:
            raise ValueError(check.reason)
        self.add_on_set_parameters_callback(self.validate)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(.1, self.publish)

    def validate(self, parameters):
        for p in parameters:
            if p.name not in self.ranges:
                continue
            lo, hi = self.ranges[p.name]
            if not isinstance(p.value, (int, float)) or not math.isfinite(p.value) or not lo <= p.value <= hi:
                return SetParametersResult(successful=False, reason=f'{p.name} must be in [{lo}, {hi}]')
        return SetParametersResult(successful=True)

    def publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for side in ('red', 'blue'):
            message.name.extend([side + '_base_target_slide_joint', side + '_dart_gate_joint'])
            message.position.extend([float(self.get_parameter(side + '_base_slide_m').value),
                                     float(self.get_parameter(side + '_gate_open_m').value)])
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SceneJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
