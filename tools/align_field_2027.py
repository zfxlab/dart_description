#!/usr/bin/env python3
"""Derive a red-side, right-corner field frame from cached RM2027 CAD meshes.

Writes a coordinate manifest and a static assembly URDF for alignment review.
Does not change mesh vertices, materials, mechanical datums or joint limits.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assets', type=Path, help='rmuc2027_v1 extraction directory')
    parser.add_argument('--config-output', type=Path, required=True)
    args = parser.parse_args()
    root = args.assets.resolve()
    inventory = json.loads((root / 'assembly_inventory.json').read_text())['nodes']
    floor = next(r for r in inventory if r['name'] == '战场大地面_带丝印')
    parts = json.loads((root / 'mesh_manifest.json').read_text())['parts']
    part = next(p for p in parts if p['definition_label'] == floor['definition_label'])
    mesh = np.load(root / part['file'])
    transform = np.array(floor['world_matrix_mm'], dtype=float)
    vertices = mesh['vertices'] @ transform[:3, :3].T + transform[:3, 3] / 1000
    # Face 184 is the positive-CAD-Y half of the main walking surface in this
    # exact extraction; check its slope and planarity before trusting the datum.
    face = vertices[np.unique(mesh['triangles'][mesh['triangle_face_indices'] == 184])]
    design = np.column_stack((face[:, :2], np.ones(len(face))))
    plane = np.linalg.lstsq(design, face[:, 2], rcond=None)[0]
    residual = float(np.max(np.abs(design @ plane - face[:, 2])))
    angle = math.atan(-plane[1])
    if residual > 1e-6 or abs(plane[0]) > 1e-8 or abs(math.degrees(angle) - 1.5) > 1e-4:
        raise ValueError('CAD floor changed; inspect the face and datum before aligning')
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    center_xy = (low[:2] + high[:2]) / 2
    # Rule figure 4-5: 28 m length; 15 m measured on the two sloping halves.
    length, slope_width = 28.0, 15.0
    projected_width = slope_width * math.cos(angle)
    origin = np.array([center_xy[0] + length / 2, center_xy[1] + projected_width / 2, 0.0])
    origin[2] = np.dot(plane, [*origin[:2], 1.0])
    rotation = np.diag([-1.0, -1.0, 1.0])
    field_from_cad = np.eye(4)
    field_from_cad[:3, :3] = rotation
    field_from_cad[:3, 3] = -rotation @ origin
    out = root / 'aligned'
    out.mkdir(exist_ok=True)
    split = json.loads((root / 'split/split_manifest.json').read_text())
    placements = []
    robot = ET.Element('robot', name='rmuc2027_alignment_review')
    ET.SubElement(robot, 'link', name='field_link')
    for asset in split['assets']:
        cad_from_local = np.eye(4)
        cad_from_local[:3, 3] = asset['local_to_cad_translation_m']
        matrix = field_from_cad @ cad_from_local
        name = asset['name'] + '_visual_link'
        link = ET.SubElement(robot, 'link', name=name)
        visual = ET.SubElement(link, 'visual')
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, 'mesh', filename=(root / 'split' / asset['file']).as_uri(), scale='1 1 1')
        joint = ET.SubElement(robot, 'joint', name='field_to_' + asset['name'], type='fixed')
        ET.SubElement(joint, 'parent', link='field_link')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(f'{x:.12g}' for x in matrix[:3, 3]), rpy=f'0 0 {math.pi:.15g}')
        placements.append({'asset': asset['file'], 'field_from_asset': matrix.tolist()})
    ET.indent(robot)
    ET.ElementTree(robot).write(out / 'field_alignment_preview.urdf', encoding='utf-8', xml_declaration=True)
    result = {
        'frame_id': 'field_link', 'units': 'meter',
        'viewpoint': 'red side looking toward blue; origin at own right field corner',
        'axes': {'x': 'forward, red to blue, CAD -X', 'y': 'left, CAD -Y', 'z': 'vertical up, CAD +Z'},
        'datum': 'rule-defined inner 28 m x 15 m rectangle centered on CAD floor; width measured on slope; z=0 at its right corner walking surface',
        'status': 'engineering datum inferred from rule dimensions and CAD symmetry, not surveyed',
        'rule_reference': 'RM2027 V1.0.0 printed pages 29, 32 (figure 4-5), 33 (figure 4-6)',
        'cad_floor_definition': floor['definition_label'], 'cad_surface_face_index': 184,
        'floor_plane_z_ax_by_c': plane.tolist(), 'floor_plane_max_residual_m': residual,
        'transverse_slope_deg': math.degrees(angle),
        'nominal_length_m': length, 'nominal_slope_width_m': slope_width,
        'projected_width_m': projected_width,
        'cad_floor_center_xy_m': center_xy.tolist(), 'origin_in_cad_m': origin.tolist(),
        'field_from_cad': field_from_cad.tolist(), 'cad_from_field': np.linalg.inv(field_from_cad).tolist(),
        'legacy_field_from_old': [[0, 1, 0, 0], [-1, 0, 0, 15], [0, 0, 1, 0], [0, 0, 0, 1]],
        'legacy_note': 'Only re-expresses the existing 15 m wide legacy map; does not register old geometry to RM2027.',
        'placements': placements,
        'preview_note': 'All links are fixed for alignment review only; these are not mechanical joint definitions.',
    }
    args.config_output.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    args.config_output.write_text(text)
    (out / 'field_alignment.json').write_text(text)
    print('Origin in CAD (m):', origin)
    print('Horizontal extent (m):', length, projected_width)
    print('Preview URDF:', out / 'field_alignment_preview.urdf')


if __name__ == '__main__':
    main()
