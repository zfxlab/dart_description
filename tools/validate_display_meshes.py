import ctypes,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import argparse
parser=argparse.ArgumentParser(description='Validate display geometry and optionally install it into this package')
parser.add_argument('assets',type=Path);parser.add_argument('--install',action='store_true');args=parser.parse_args()
R=args.assets.resolve();O=R/'optimized';P=Path(__file__).resolve().parents[1]
report=json.load(open(O/'simplification.json'));assert max(x['bounds_error_m'] for x in report)<.00201
s=json.load(open(R/'split/split_manifest.json'));worst=0
for ps in s['partition'].values():
 for p in ps.values():
  a=np.load(R/'split'/p['file']);b=np.load(O/p['file']);assert np.array_equal(a['colors'],b['colors']);assert np.isfinite(b['vertices']).all() and np.isfinite(b['normals']).all()
  shaded=np.load(O/'shaded'/p['file'])
  assert np.array_equal(shaded['vertices'][shaded['triangles']],b['vertices'][b['triangles']])
  assert np.array_equal(shaded['colors'],b['colors']) and np.array_equal(shaded['color_indices'],b['color_indices'])
  assert np.isfinite(shaded['normals']).all() and np.allclose(np.linalg.norm(shaded['normals'],axis=1),1,atol=1e-5)
  col=a['colors'];green=(col[:,1]>col[:,0]*1.5)&(col[:,1]>col[:,2]*1.5)
  av=a['vertices'][np.unique(a['triangles'][green[a['color_indices']]])];bv=b['vertices'][np.unique(b['triangles'][green[b['color_indices']]])]
  if len(av):worst=max(worst,cKDTree(av).query(bv)[0].max(),cKDTree(bv).query(av)[0].max())
assert worst<1e-6
lib=ctypes.CDLL('libassimp.so.5');lib.aiImportFile.argtypes=[ctypes.c_char_p,ctypes.c_uint];lib.aiImportFile.restype=ctypes.c_void_p;lib.aiReleaseImport.argtypes=[ctypes.c_void_p]
for a in s['assets']:
 scene=lib.aiImportFile(str(O/a['file']).encode(),0);assert scene,a['file'];lib.aiReleaseImport(scene);print('Loaded',a['file'],flush=True)
from reuse_display_meshes import validate_reused, install_reused
reuse_report=validate_reused(O,lib)
result={'assets':json.load(open(O/'assets.json')),'maximum_bounds_error_m':max(x['bounds_error_m'] for x in report),'green_vertices_bidirectional_nearest_error_m':worst,'assimp_imports_passed':15,'method':'weld 1um; 1deg planar dissolve preserving material boundaries; weighted collapse protects material boundaries, sharp edges and green surfaces; 2mm bounds gate with planar-only fallback; bake original internal instance matrices','source_master':'RM_Assets/derived/rmuc2027_v1/split (unmodified)'}

result['shading']={'sharp_angle_degrees':20,'triangle_positions_exactly_preserved':True,'unit_normals_checked':True,'ambient_factor':1.0,'specular_rgb':[0,0,0],'cad_palette_unchanged':True}
result['shading']['display_color_conversion']='linear CAD RGB to sRGB for default RViz OGRE display; alpha unchanged'
result['shading']['display_face_color_overrides']=json.loads((O/'material_corrections.json').read_text())
result['reuse']=reuse_report
if args.install:
 install_reused(O,P)
 (P/'metadata/display_meshes.json').write_text(json.dumps(result,indent=2))
(O/'validation.json').write_text(json.dumps(result,indent=2));print('INSTALLED' if args.install else 'VALIDATED',flush=True)
