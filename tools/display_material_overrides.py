"""Apply audited CAD-face color corrections to display meshes, leaving masters intact."""
import numpy as np


def apply_face_colors(key, part, source, rules):
    result = dict(part)
    palette = [list(c) for c in part['colors']]
    indices = part['color_indices'].copy()
    points = part['vertices'][part['triangles']]
    report = []
    for rule in rules:
        if rule['mesh_key'] != key:
            continue
        color = rule['replacement_rgba']
        if color not in palette:
            palette.append(color)
        replacement = palette.index(color)
        for face in rule['source_faces']:
            original = source['vertices'][source['triangles'][source['triangle_face_indices'] == face]]
            if not len(original):
                raise ValueError(f'{key}: missing source face {face}')
            flat = original.reshape(-1, 3)
            center = flat.mean(axis=0)
            normal = np.cross(original[:, 1] - original[:, 0], original[:, 2] - original[:, 0]).sum(axis=0)
            normal /= np.linalg.norm(normal)
            tolerance = 5e-5  # 0.05 mm, allowing float32 display coordinates
            assert np.max(np.abs((flat - center) @ normal)) < tolerance
            # Simplification preserves material boundaries. Restrict to the original
            # gray material, face plane and bounds, so other metalwork stays gray.
            mask = np.all(np.isclose(part['colors'][part['color_indices']], rule['match_rgba'], atol=1e-6), axis=1)
            mask &= np.all(np.abs((points - center) @ normal) < tolerance, axis=1)
            mask &= np.all((points >= flat.min(0) - tolerance) &
                           (points <= flat.max(0) + tolerance), axis=(1, 2))
            if not mask.any():
                raise ValueError(f'{key}: no display triangles match face {face}')
            source_area = np.linalg.norm(np.cross(original[:, 1] - original[:, 0],
                                                  original[:, 2] - original[:, 0]), axis=1).sum() / 2
            selected = points[mask]
            display_area = np.linalg.norm(np.cross(selected[:, 1] - selected[:, 0],
                                                   selected[:, 2] - selected[:, 0]), axis=1).sum() / 2
            if abs(display_area / source_area - 1) > .01:
                raise ValueError(f'{key}: incomplete or ambiguous face {face} match')
            indices[mask] = replacement
            report.append({'mesh_key': key, 'source_face': face, 'triangles': int(mask.sum()),
                           'source_area_m2': float(source_area), 'display_area_m2': float(display_area)})
    result['colors'] = np.array(palette)
    result['color_indices'] = indices
    return result, report
