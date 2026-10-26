"""Public analysis API regressions, independent of mesh rendering."""
from pathlib import Path
import math
import sys

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from target_geometry import Geometry


@pytest.mark.parametrize('yaw', [0.0, -0.2])
def test_ballistic_reference_uses_release_origin_and_launcher_axes(yaw):
    g = Geometry()
    q = {'launcher_yaw_joint': yaw}
    ballistic = g.pose('ballistic_reference_frame', q)
    release = g.pose('dart_release_link', q)
    launcher = g.pose('launcher_frame', q)
    np.testing.assert_allclose(ballistic[:3, 3], release[:3, 3], atol=1e-12)
    np.testing.assert_allclose(ballistic[:3, :3], launcher[:3, :3], atol=1e-12)
    assert not np.allclose(ballistic[:3, :3], release[:3, :3])


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_public_queries_across_slide_and_yaw(team):
    g = Geometry(team=team)
    for slide in [*g.slide_limits, 0.0]:
        for frame in ['field_link', 'dart_base_link', 'left_camera_optical_frame', 'right_camera_optical_frame']:
            for target, feature, link in [('base','green','green_light_link'), ('base','armor','armor_link'),
                                           ('outpost','green','outpost_green_light_link'), ('outpost','armor','outpost_armor_link')]:
                p = g.target_position(target, feature, slide, reference=frame)
                expected = g.pose(link, {g.slide_joint:slide}, reference=frame)[:3,3]
                np.testing.assert_allclose(p, expected, atol=1e-10)
            np.testing.assert_allclose(g.green_position(slide, frame),g.target_position('base','green',slide,frame))
            np.testing.assert_allclose(g.armor_position(slide, frame),g.target_position('base','armor',slide,frame))
            np.testing.assert_allclose(g.module_position(slide, frame),g.position('dart_detection_module_link',slide,reference=frame))
        delta=g.armor_position(slide)-g.launcher_origin()
        assert g.target_yaw(slide)==pytest.approx(math.atan2(delta[1],delta[0]))
        assert g.horizontal_distance(slide)==pytest.approx(np.linalg.norm(delta[:2]))
    for yaw in [*g.joint_limits('launcher_yaw_joint'), -.1]:
        for frame in ['field_link', 'dart_base_link', 'left_camera_optical_frame']:
            pose=g.release_pose(yaw,frame)
            np.testing.assert_allclose(g.release_position(yaw,frame),pose[:3,3])
            direction=g.launch_direction(yaw,frame)
            np.testing.assert_allclose(direction,pose[:3,0])
            assert np.linalg.norm(direction)==pytest.approx(1.)
            np.testing.assert_allclose(pose,g.pose('dart_release_link',{'launcher_yaw_joint':yaw},reference=frame),atol=1e-10)
    # Rigid transform composition must hold even with multiple moving joints.
    q={g.slide_joint:.1,'launcher_yaw_joint':-.1,'red_dart_gate_joint':.6}
    a,b='armor_link','left_camera_optical_frame'
    np.testing.assert_allclose(g.pose(b,q)@g.pose(a,q,reference=b),g.pose(a,q),atol=1e-10)


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_surface_coordinates_edges_and_outside(team):
    g=Geometry(team=team)
    for frame in ['field_link','dart_base_link','left_camera_optical_frame']:
        for s in [g.armor_surface(.1,frame),g.outpost_armor_surface(frame)]:
            basis=np.column_stack([s.normal,s.axis_u,s.axis_v])
            np.testing.assert_allclose(basis.T@basis,np.eye(3),atol=1e-12)
            assert np.linalg.det(basis)==pytest.approx(1.)
            assert s.contains(s.center)
            for u in [-s.size_u/2,s.size_u/2]:
                for v in [-s.size_v/2,s.size_v/2]:
                    point=s.center+u*s.axis_u+v*s.axis_v
                    np.testing.assert_allclose(s.coordinates(point),[u,v,0],atol=1e-12)
                    assert s.contains(point)
            assert not s.contains(s.center+(s.size_u/2+1e-5)*s.axis_u)
            assert not s.contains(s.center+1e-5*s.normal)
        surface=g.armor_surface(.1,frame)
        np.testing.assert_allclose(g.surface_coordinates(surface.center,.1,frame),[0,0,0],atol=1e-12)


@pytest.mark.parametrize('positions', [
    {'launcher_yaw_joint':.01}, {'red_dart_gate_joint':1.201},
    {'blue_base_target_slide_joint':-.281}, {'typo_joint':0},
    {'red_dart_gate_joint':float('nan')}, {'red_dart_gate_joint':float('inf')},
])
def test_pose_rejects_invalid_joint_states_even_outside_requested_branch(positions):
    with pytest.raises(ValueError):Geometry().pose('armor_link',positions)


def test_invalid_inputs():
    with pytest.raises(ValueError):Geometry(team='unknown')
    g=Geometry()
    for frame in ['missing_link','missing_reference']:
        with pytest.raises(ValueError):g.pose(frame)
        with pytest.raises(ValueError):g.pose('armor_link',reference=frame)
    with pytest.raises(ValueError):g.target_position('unknown')
    with pytest.raises(ValueError):g.target_position(feature='unknown')
    with pytest.raises(ValueError):g.green_position(float('nan'))
    with pytest.raises(ValueError):g.release_pose(.01)
