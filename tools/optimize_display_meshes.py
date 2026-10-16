"""Run with blender -b --python SCRIPT -- /path/to/rmuc2027_v1."""
import json,math,sys
from pathlib import Path
import bpy,bmesh,numpy as np
import argparse
parser=argparse.ArgumentParser(description='Build color-preserving display NPZ meshes with Blender')
parser.add_argument('assets',type=Path)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
R=args.assets.resolve();O=R/'optimized';O.mkdir(exist_ok=True);(O/'meshes').mkdir(exist_ok=True)
s=json.load(open(R/'split/split_manifest.json'));parts={p['mesh_key']:p for d in s['partition'].values() for p in d.values()};report=[]
for key,p in parts.items():
 a=np.load(R/'split'/p['file']);v=a['vertices'];t=a['triangles'];mesh=bpy.data.meshes.new(key);mesh.from_pydata(v.tolist(),[],t.tolist());mesh.update()
 for c in a['colors']:
  mat=bpy.data.materials.new('CAD');mat.diffuse_color=tuple(c);mesh.materials.append(mat)
 mesh.polygons.foreach_set('material_index',a['color_indices'].astype(np.int32));obj=bpy.data.objects.new(key,mesh);bpy.context.collection.objects.link(obj);bpy.context.view_layer.objects.active=obj;obj.select_set(True)
 bm=bmesh.new();bm.from_mesh(mesh);bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-6);bm.to_mesh(mesh);bm.free()
 planar=obj.modifiers.new('Planar','DECIMATE');planar.decimate_type='DISSOLVE';planar.angle_limit=math.radians(1);planar.delimit={'MATERIAL'};bpy.ops.object.modifier_apply(modifier=planar.name)
 # Pin material boundaries and green light geometry with a weighted collapse.
 mesh=obj.data;bm=bmesh.new();bm.from_mesh(mesh);bm.verts.ensure_lookup_table()
 protected=set()
 for e in bm.edges:
  if len(e.link_faces)!=2 or len({f.material_index for f in e.link_faces})>1 or e.calc_face_angle(0.0)>math.radians(20):protected.update(v.index for v in e.verts)
 for f in bm.faces:
  c=a['colors'][f.material_index]
  if c[1]>c[0]*1.5 and c[1]>c[2]*1.5:protected.update(v.index for v in f.verts)
 bm.free()
 vg=obj.vertex_groups.new(name='simplifiable');free=list(set(range(len(mesh.vertices)))-protected)
 if free:vg.add(free,1.0,'REPLACE')
 # Keep printed floor graphics planar-only. Other solids target 15%; targets 40%.
 floor=key.startswith('0:1:1:2_')
 backup=obj.data.copy();fallback=False
 if not floor and free and len(t)>2000:
  dec=obj.modifiers.new('Display LOD','DECIMATE');dec.decimate_type='COLLAPSE';dec.ratio=.4 if key.endswith('_target') else .15;dec.use_collapse_triangulate=True;dec.vertex_group=vg.name;dec.invert_vertex_group=True;dec.vertex_group_factor=1000.;bpy.ops.object.modifier_apply(modifier=dec.name)
 check=np.array([x.co[:] for x in obj.data.vertices])
 if max(np.abs(check.min(0)-v.min(0)).max(),np.abs(check.max(0)-v.max(0)).max())>.002:
  obj.data=backup;fallback=True
 mesh=obj.data;mesh.calc_loop_triangles();vs=np.array([x.co[:] for x in mesh.vertices],dtype=np.float32);ts=np.array([x.vertices[:] for x in mesh.loop_triangles],dtype=np.int32);ci=np.array([mesh.polygons[x.polygon_index].material_index for x in mesh.loop_triangles],dtype=np.int32);ns=np.array([x.normal[:] for x in mesh.vertices],dtype=np.float32)
 colors=a['colors'];green=(colors[:,1]>colors[:,0]*1.5)&(colors[:,1]>colors[:,2]*1.5)
 if green.any():
  keep=~green[ci];src=green[a['color_indices']];used,inv=np.unique(a['triangles'][src],return_inverse=True);ts=np.concatenate([ts[keep],inv.reshape(-1,3)+len(vs)]);ci=np.concatenate([ci[keep],a['color_indices'][src]]);vs=np.concatenate([vs,a['vertices'][used]]);ns=np.concatenate([ns,a['normals'][used]])
 assert len(ts)>0 and np.isfinite(vs).all()
 np.savez_compressed(O/p['file'],vertices=vs,triangles=ts,normals=ns,colors=a['colors'],color_indices=ci)
 report.append({'key':key,'before':len(t),'after':len(ts),'protected_vertices':len(protected),'planar_only_fallback':fallback,'bounds_error_m':float(max(np.abs(vs.min(0)-v.min(0)).max(),np.abs(vs.max(0)-v.max(0)).max()))})
 print(key,len(t),'->',len(ts),flush=True);bpy.data.objects.remove(obj,do_unlink=True)
json.dump(report,open(O/'simplification.json','w'),indent=2)
