"""Frame, material-placement and motion regressions for the RM2027 replacement."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / 'tools'))
from target_geometry import Geometry
from target_geometry.core import transform


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_all_colored_parts_reassemble_at_cad_pose(team):
    g = Geometry(team=team)
    audit = json.loads((PACKAGE / 'metadata/rm2027_mechanisms.json').read_text())
    links = {e.getAttribute('name'): e for e in g.document.getElementsByTagName('link')}
    reuse = json.loads((PACKAGE / 'metadata/mesh_reuse.json').read_text())
    assert len(audit['visuals']) == sum(len(a['instances']) for a in reuse['assets'].values())
    for item in audit['visuals']:
        visual = next(v for v in links[item['link']].getElementsByTagName('visual')
                      if v.getAttribute('name') == item['visual_name'])
        o = visual.getElementsByTagName('origin')[0]
        m = transform([float(x) for x in o.getAttribute('xyz').split()],
                      [float(x) for x in o.getAttribute('rpy').split()])
        np.testing.assert_allclose(g.pose(item['link']) @ m,
                                   item['field_from_asset_zero'], atol=1e-9)
        assert not visual.getElementsByTagName('material')  # preserve DAE colors
        mesh = visual.getElementsByTagName('mesh')[0].getAttribute('filename')
        assert (PACKAGE / mesh.removeprefix('package://dart_description/')).is_file()
    for link in links:
        assert np.isfinite(g.pose(link)).all()  # connected, acyclic TF tree


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_active_target_and_slide_direction(team):
    g = Geometry(team=team)
    opponent = 'blue' if team == 'red' else 'red'
    np.testing.assert_allclose(g.pose('green_light_link'), g.pose(opponent + '_base_green_light_link'))
    light_normal = g.pose('green_light_link')[:3, 0]
    assert light_normal[2] == pytest.approx(math.sin(math.radians(14.5)), abs=1e-8)
    expected_sign = -1 if opponent == 'blue' else 1
    delta = g.green_position(.28, reference='field_link') - g.green_position(-.28, reference='field_link')
    np.testing.assert_allclose(delta, [0, .56 * expected_sign, 0], atol=1e-9)
    for q in [-.28, 0, .28]:
        rigid = g.pose('armor_link', {g.slide_joint: q}, reference='green_light_link')
        np.testing.assert_allclose(rigid, g.pose('armor_link', reference='green_light_link'), atol=1e-10)
        np.testing.assert_allclose(g.target_position('outpost', slide=q), g.target_position('outpost'), atol=1e-10)
    assert g.armor_surface(0).contains(g.armor_position(0))
    with pytest.raises(ValueError):
        g.green_position(.281)


def test_gates_open_upwards_from_lower_closed_pose():
    g = Geometry()
    for side in ['red', 'blue']:
        frame = side + '_dart_gate_link'
        shift = g.pose(frame, {side + '_dart_gate_joint': 1.2})[:3, 3] - g.pose(frame)[:3, 3]
        assert shift[2] > 1.1
        assert np.linalg.norm(shift) == pytest.approx(1.2)
        assert abs(shift[0]) == pytest.approx(1.2 * math.sin(math.pi / 12), abs=1e-8)
        np.testing.assert_allclose(g.pose(side + '_dart_station_link', {side + '_dart_gate_joint': 1.2}),
                                   g.pose(side + '_dart_station_link'))
        # CAD is not an endpoint: the exact bracket contact is 53.25 mm above it.
        axis = np.array([-math.sin(math.pi/12), 0, math.cos(math.pi/12)])
        opened = g.pose(frame, {side + '_dart_gate_joint': 1.2},
                        reference=side + '_dart_station_link')
        np.testing.assert_allclose(opened[:3, 3], .05325 * axis, atol=1e-9)
        root = ET.fromstring(g.document.toxml())
        visual = root.find(f"link[@name='{frame}']/visual/origin")
        local = transform(list(map(float, visual.get('xyz').split())),
                          list(map(float, visual.get('rpy').split())))
        audit = json.loads((PACKAGE/'metadata/rm2027_mechanisms.json').read_text())
        entry = next(a for a in audit['visuals'] if a['link'] == frame)
        np.testing.assert_allclose(g.pose(frame, {side + '_dart_gate_joint': 1.2}) @ local,
                                   entry['field_from_asset_open'], atol=1e-9)
        contact = json.loads((PACKAGE/'metadata/gate_contact.json').read_text())
        moving = np.array(contact['moving']['center']) + opened[:3, 3]
        fixed = np.array(contact['fixed']['center'])
        normal = np.array(contact['fixed']['normal'])
        assert abs((moving-fixed) @ normal) < 1e-9
        assert contact['common_area_mm2'] > 100


def test_camera_and_launcher_rig_is_same_for_both_teams():
    red, blue = Geometry(team='red'), Geometry(team='blue')
    for frame in ['left_camera_optical_frame', 'right_camera_optical_frame', 'dart_release_link']:
        np.testing.assert_allclose(red.pose(frame, reference='dart_base_link'),
                                   blue.pose(frame, reference='dart_base_link'), atol=1e-10)
    assert red.pose('dart_base_link')[0, 3] < 2
    assert blue.pose('dart_base_link')[0, 3] > 26
    assert red.target_position('base', reference='field_link')[0] > 25
    assert blue.target_position('base', reference='field_link')[0] < 3


def test_geometry_constants_are_defined_in_modules():
    doc = xacro.process_file(str(PACKAGE/'urdf/dart_system.urdf.xacro'))
    root = ET.fromstring(doc.toxml())
    assert root.find("joint[@name='red_dart_gate_joint']/limit").get('upper') == '1.2'
    assert root.find("joint[@name='dart_pedestal_to_base']/origin").get('xyz') == '0 0 0'
    for path in (PACKAGE/'urdf').rglob('*.xacro'):
        for arg in ET.parse(path).getroot().iter('{http://www.ros.org/wiki/xacro}arg'):
            assert arg.get('name') == 'team'


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_launcher_mount_on_selected_lower_chamber_face(team):
    g = Geometry(team=team)
    for side in ['red', 'blue']:
        m = g.pose(side + '_launcher_mount_link', reference=side + '_dart_station_link')
        np.testing.assert_allclose(m[:3, 3], [-.1385, 0, .055], atol=1e-9)
        np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-9)
    np.testing.assert_allclose(g.pose('dart_base_link'),
                               g.pose(team + '_launcher_mount_link'), atol=1e-9)


@pytest.mark.parametrize('team', ['red', 'blue'])
def test_armor_origins_on_outer_contact_surfaces(team):
    g = Geometry(team=team)
    audit = json.loads((PACKAGE/'metadata/rm2027_mechanisms.json').read_text())
    for side in ['red', 'blue']:
        for kind, definition, face, expected in [
            ('base', '0:1:1:64', 140, [0, 1.12334119, -.002741312]),
            ('outpost', '0:1:1:16', 17250, [.06446706476128, 1.4158055920155, -.001131144231245]),
        ]:
            data = audit['targets'][side+'_'+kind]
            assert (data['armor_definition'], data['armor_face']) == (definition, face)
            mount = np.array(data['armor_frame_definition'])
            np.testing.assert_allclose(mount[:3, 3], expected, atol=1e-8)
            assert mount[1, 0] > .88  # outward/upward face, not rear surface
            green = np.eye(4)
            green[:3, :3] = data['green_rotation_definition']
            green[:3, 3] = data['green_center_definition_m']
            np.testing.assert_allclose(
                g.pose(side+'_'+kind+'_armor_link', reference=side+'_'+kind+'_green_light_link'),
                np.linalg.inv(green) @ mount, atol=1e-9)


def test_face_color_override_leaves_other_surfaces_unchanged():
    from display_material_overrides import apply_face_colors
    v = np.array([[0.,0,0], [1,0,0], [0,1,0], [0,0,1], [1,0,1], [0,1,1]])
    t = np.array([[0,1,2], [3,4,5], [0,1,2]])
    colors = np.array([[.65,.65,.65,1], [1,0,0,1]])
    part = {'vertices':v, 'triangles':t, 'colors':colors, 'color_indices':np.array([0,0,1])}
    source = {**part, 'triangles':t[:1], 'triangle_face_indices':np.array([42])}
    rules = [{'mesh_key':'test', 'source_faces':[42], 'match_rgba':colors[0].tolist(),
              'replacement_rgba':[.0331048,.0331048,.0331048,1]}]
    result, report = apply_face_colors('test', part, source, rules)
    assert result['color_indices'].tolist() == [2,0,1]
    assert part['color_indices'].tolist() == [0,0,1]
    np.testing.assert_array_equal(result['vertices'], v)
    assert report[0]['triangles'] == 1


def test_shared_meshes_are_reused_and_all_referenced():
    root = ET.fromstring(Geometry().document.toxml())
    def meshes(link):
        return [m.get('filename') for m in root.findall(f"link[@name='{link}']/visual/geometry/mesh")]
    for suffix in ['base_link', 'rail_link', 'base_target_link', 'outpost_link',
                   'outpost_target_link', 'dart_station_link', 'dart_gate_link']:
        assert meshes('red_'+suffix) == meshes('blue_'+suffix)
    paths = {m.get('filename') for m in root.findall('link/visual/geometry/mesh')
             if '/shared/' in m.get('filename')}
    files = {'package://dart_description/meshes/shared/'+p.name
             for p in (PACKAGE/'meshes/shared').glob('*.dae')}
    assert paths == files
    audit = json.loads((PACKAGE/'metadata/rm2027_mechanisms.json').read_text())
    assert len(paths) < len(audit['visuals'])


@pytest.mark.parametrize('yaw', [-math.pi/9, -.1, 0])
def test_runtime_rig_is_scene_independent_and_matches_scene(yaw):
    path = PACKAGE/'urdf/dart_robot.urdf.xacro'
    root = ET.fromstring(xacro.process_file(str(path)).toxml())
    from target_geometry.core import axis_rotation
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    def rig_pose(link):
        if link == "dart_base_link":return np.eye(4)
        joint = joints[link];o = joint.find("origin")
        m = transform(list(map(float, o.get("xyz").split())),
                      list(map(float, o.get("rpy").split())))
        if joint.get("name") == "launcher_yaw_joint":
            m = m @ axis_rotation([0,0,1], yaw)
        return rig_pose(joint.find("parent").get("link")) @ m
    links = {e.get('name') for e in root.findall('link')}
    children = {e.find('child').get('link') for e in root.findall('joint')}
    assert links-children == {'dart_base_link'}
    assert len(links) == 9
    assert 'ballistic_reference_frame' in links
    assert {e.get('name') for e in root.findall('joint') if e.get('type') != 'fixed'} == {'launcher_yaw_joint'}
    assert all('/shared/' not in e.get('filename') for e in root.iter('mesh'))
    for team in ['red', 'blue']:
        scene = Geometry(team=team)
        for link in links:
            np.testing.assert_allclose(
                rig_pose(link),
                scene.pose(link, {'launcher_yaw_joint':yaw}, reference='dart_base_link'), atol=1e-10)
    # Team selection cannot affect the standalone rig.
    changed = xacro.process_file(str(path), mappings={'team':'blue'})
    assert ET.tostring(ET.fromstring(changed.toxml())) == ET.tostring(root)
