"""Rebuild split normals without changing geometry. Run using Blender --python."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np


def split_normals(vertices, triangles, angle_degrees=20):
    mesh = bpy.data.meshes.new('display_normals')
    try:
        mesh.from_pydata(vertices.tolist(), [], triangles.tolist())
        mesh.update()
        mesh.polygons.foreach_set('use_smooth', np.ones(len(triangles), dtype=bool))
        mesh.set_sharp_from_angle(angle=math.radians(angle_degrees))
        normals = np.empty((len(mesh.loops), 3), dtype=np.float32)
        mesh.corner_normals.foreach_get('vector', normals.ravel())
        lengths = np.linalg.norm(normals, axis=1)
        invalid = lengths < 1e-12
        if invalid.any():
            face = np.cross(vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
                            vertices[triangles[:, 2]] - vertices[triangles[:, 0]])
            normals[invalid] = np.repeat(face, 3, axis=0)[invalid]
            lengths = np.linalg.norm(normals, axis=1)
            # Zero-area triangles are removed by the exporter; keep finite normals here.
            normals[lengths < 1e-12] = (0, 0, 1)
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        # Share only corners with both identical position indices and normals.
        # Rounding removes insignificant floating point differences on planar faces.
        keys = np.empty(len(normals), dtype=[('vertex', '<i4'), ('normal', '<f4', (3,))])
        keys['vertex'] = triangles.ravel()
        keys['normal'] = np.round(normals, 6)
        _, first, inverse = np.unique(keys, return_index=True, return_inverse=True)
        source_indices = triangles.ravel()[first]
        return vertices[source_indices], inverse.reshape(-1, 3).astype(np.int32), normals[first]
    finally:
        bpy.data.meshes.remove(mesh)


def check_corner_regression():
    # Two perpendicular quads: shared corners must not acquire averaged normals.
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                  [0, 0, 1], [1, 0, 1]], dtype=np.float32)
    t = np.array([[0, 1, 2], [0, 2, 3], [0, 4, 5], [0, 5, 1]], dtype=np.int32)
    sv, st, sn = split_normals(v, t)
    face = np.cross(sv[st[:, 1]] - sv[st[:, 0]], sv[st[:, 2]] - sv[st[:, 0]])
    face /= np.linalg.norm(face, axis=1)[:, None]
    assert np.all(np.einsum('tci,ti->tc', sn[st], face) > .99999)
    assert np.array_equal(sv[st], v[t])
    # A shallow bend must still shade smoothly across its shared edge.
    v[4:] = [[0, -1, .1], [1, -1, .1]]
    sv, st, sn = split_normals(v, t)
    assert len(sv) == len(v)
    print('PASS: hard corner and smooth bend regressions', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assets', type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    root = args.assets.resolve()
    output = root / 'optimized/shaded'
    output.mkdir(parents=True, exist_ok=True)
    check_corner_regression()
    manifest = json.loads((root / 'split/split_manifest.json').read_text())
    parts = {p['mesh_key']: p for group in manifest['partition'].values() for p in group.values()}
    report = []
    for key, part in parts.items():
        with np.load(root / 'optimized' / part['file']) as source:
            v, t, n = split_normals(source['vertices'], source['triangles'])
            assert np.array_equal(v[t], source['vertices'][source['triangles']])
            assert np.isfinite(n).all()
            lengths = np.linalg.norm(n, axis=1)
            assert np.all((lengths > .999) & (lengths < 1.001))
            destination = output / part['file']
            destination.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(destination, vertices=v, triangles=t, normals=n,
                                colors=source['colors'], color_indices=source['color_indices'])
            report.append({'key': key, 'triangles': len(t),
                           'source_sha256': hashlib.sha256((root / 'optimized' / part['file']).read_bytes()).hexdigest(),
                           'vertices_before': len(source['vertices']), 'vertices_after': len(v),
                           'triangle_positions_exactly_preserved': True})
            print(key, report[-1], flush=True)
    (output / 'shading.json').write_text(json.dumps({
        'sharp_angle_degrees': 20, 'parts': report,
        'regressions': ['perpendicular planes stay flat', 'shallow bend stays smooth'],
    }, indent=2))


if __name__ == '__main__':
    main()
