import hashlib,json,sys
from pathlib import Path
import numpy as np
import argparse,xml.etree.ElementTree as ET
from display_material_overrides import apply_face_colors
parser=argparse.ArgumentParser(description='Export baked, color-preserving display COLLADA files')
parser.add_argument('assets',type=Path);args=parser.parse_args();R=args.assets.resolve();O=R/'optimized'
def display_color(linear_rgba):
    # STEP extraction stores linear RGB. RViz's default OGRE window does not
    # request gamma correction; use sRGB display values for its material colors.
    color=np.array(linear_rgba,dtype=float,copy=True)
    rgb=np.clip(color[:3],0,1)
    color[:3]=np.where(rgb<=0.0031308,12.92*rgb,1.055*np.power(rgb,1/2.4)-0.055)
    return color

assert np.allclose(display_color([0.03310476657]*3+[1]),[.2,.2,.2,1],atol=1e-7)
assert np.allclose(display_color([0,1,0,.4]),[0,1,0,.4])
def write_dae(path,meshes,instances):
    root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
    effects=ET.SubElement(root,'library_effects');materials=ET.SubElement(root,'library_materials');geometries=ET.SubElement(root,'library_geometries')
    used={r.get('mesh_key',r['definition_label']) for r in instances}
    for key,m in meshes.items():
        if key not in used:continue
        gid=f'g{m["index"]}'
        for j,color in enumerate(m['palette']):
            color=display_color(color)
            mid=f'{gid}m{j}';effect=ET.SubElement(effects,'effect',id=mid+'fx');profile=ET.SubElement(effect,'profile_COMMON');tech=ET.SubElement(profile,'technique',sid='common');phong=ET.SubElement(tech,'phong')
            ET.SubElement(ET.SubElement(phong,'diffuse'),'color').text=' '.join(map(str,color))
            # RViz's Assimp loader applies another 0.5 factor to ambient.
            # Preserve CAD colors; avoid the previous extra 0.3 attenuation.
            ET.SubElement(ET.SubElement(phong,'ambient'),'color').text=' '.join(map(str,color))
            ET.SubElement(ET.SubElement(phong,'specular'),'color').text='0 0 0 1'
            mat=ET.SubElement(materials,'material',id=mid);ET.SubElement(mat,'instance_effect',url='#'+mid+'fx')
        geom=ET.SubElement(geometries,'geometry',id=gid);mesh=ET.SubElement(geom,'mesh')
        for suffix,data in [('pos',m['v']),('norm',m['n'])]:
            sid=gid+suffix;src=ET.SubElement(mesh,'source',id=sid)
            ET.SubElement(src,'float_array',id=sid+'array',count=str(data.size)).text=' '.join(f'{x:.8g}' for x in data.ravel())
            acc=ET.SubElement(ET.SubElement(src,'technique_common'),'accessor',source='#'+sid+'array',count=str(len(data)),stride='3')
            for component in 'XYZ':ET.SubElement(acc,'param',name=component,type='float')
        vert=ET.SubElement(mesh,'vertices',id=gid+'verts');ET.SubElement(vert,'input',semantic='POSITION',source='#'+gid+'pos')
        for j in range(len(m['palette'])):
            indices=m['t'][m['ci']==j]
            tri=ET.SubElement(mesh,'triangles',count=str(len(indices)),material=f'{gid}m{j}')
            ET.SubElement(tri,'input',semantic='VERTEX',source='#'+gid+'verts',offset='0');ET.SubElement(tri,'input',semantic='NORMAL',source='#'+gid+'norm',offset='0')
            ET.SubElement(tri,'p').text=' '.join(map(str,indices.ravel()))
    vs=ET.SubElement(ET.SubElement(root,'library_visual_scenes'),'visual_scene',id='scene')
    for r in instances:
        m=meshes[r.get('mesh_key',r['definition_label'])];gid=f'g{m["index"]}'
        node=ET.SubElement(vs,'node',id=f'node{r["id"]}',name=r['name'] or r['definition_name'])
        matrix=np.array(r['world_matrix_mm'],dtype=float);matrix[:3,3]*=0.001
        ET.SubElement(node,'matrix').text=' '.join(f'{x:.10g}' for x in matrix.ravel())
        inst=ET.SubElement(node,'instance_geometry',url='#'+gid)
        common=ET.SubElement(ET.SubElement(inst,'bind_material'),'technique_common')
        for j in range(len(m['palette'])):ET.SubElement(common,'instance_material',symbol=f'{gid}m{j}',target=f'#{gid}m{j}')
    ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#scene')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    print('Wrote',path.name,flush=True)

s=json.load(open(R/'split/split_manifest.json'));parts={p['mesh_key']:p for d in s['partition'].values() for p in d.values()}
if not (O/'shaded/shading.json').exists():
 raise SystemExit('Run shade_display_meshes.py with Blender before exporting.')
shading=json.loads((O/'shaded/shading.json').read_text())
for row in shading['parts']:
 if hashlib.sha256((O/parts[row['key']]['file']).read_bytes()).hexdigest()!=row['source_sha256']:
  raise SystemExit('Shaded cache is stale; rerun shade_display_meshes.py.')
cache={k:dict(np.load(O/'shaded'/p['file'])) for k,p in parts.items()};report=[]
rules=json.loads((Path(__file__).resolve().parents[1]/'metadata/display_material_overrides.json').read_text())['rules']
corrections=[]
for key in {rule['mesh_key'] for rule in rules}:
 cache[key],changed=apply_face_colors(key,cache[key],dict(np.load(R/'split'/parts[key]['file'])),rules)
 corrections.extend(changed)
(O/'material_corrections.json').write_text(json.dumps(corrections,indent=2))
for asset in s['assets']:
 vs=[];ts=[];ns=[];cs=[];palette=[];offset=0
 for ins in asset['instances']:
  a=cache[ins['mesh_key']];m=np.array(ins['world_matrix_mm']);v=a['vertices']@m[:3,:3].T+m[:3,3]*.001;n=a['normals']@m[:3,:3].T
  remap=[]
  for c in a['colors']:
   c=tuple(c)
   if c not in palette:palette.append(c)
   remap.append(palette.index(c))
  vs.append(v);ns.append(n);ts.append(a['triangles']+offset);cs.append(np.array(remap)[a['color_indices']]);offset+=len(v)
 v=np.concatenate(vs);t=np.concatenate(ts);n=np.concatenate(ns);ci=np.concatenate(cs);pal=np.array(palette)
 cross=np.cross(v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]]);valid=np.linalg.norm(cross,axis=1)>1e-12;t=t[valid];ci=ci[valid]
 meshes={'combined':{'index':0,'v':v,'t':t,'n':n,'palette':pal,'ci':ci}};inst={'id':0,'name':asset['name'],'definition_label':'combined','world_matrix_mm':np.eye(4).tolist()}
 write_dae(O/asset['file'],meshes,[inst])
 report.append({'file':asset['file'],'before_triangles':asset['triangles'],'after_triangles':len(t),'before_bytes':(R/'split'/asset['file']).stat().st_size,'after_bytes':(O/asset['file']).stat().st_size,'material_colors':len(pal)})
json.dump(report,open(O/'assets.json','w'),indent=2)
print('Triangles:',sum(a['before_triangles'] for a in report),'->',sum(a['after_triangles'] for a in report));print('Bytes:',sum(a['before_bytes'] for a in report),'->',sum(a['after_bytes'] for a in report))

# Keep definition-local geometry once; instance placement belongs to URDF.
shared=O/'reused';shared.mkdir(exist_ok=True)
reuse={'parts':{},'assets':{}}
for key in sorted({i['mesh_key'] for asset in s['assets'] for i in asset['instances']}):
 a=cache[key];v=a['vertices'].astype(float);t=a['triangles']
 valid=np.linalg.norm(np.cross(v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]]),axis=1)>1e-12
 filename='part_'+key.replace(':','_')+'.dae'
 m={'index':0,'v':v,'t':t[valid],'n':a['normals'],'palette':a['colors'],'ci':a['color_indices'][valid]}
 write_dae(shared/filename,{'combined':m},[{'id':0,'name':key,'definition_label':'combined','world_matrix_mm':np.eye(4).tolist()}])
 reuse['parts'][key]={'file':filename,'triangles':int(valid.sum()),'bytes':(shared/filename).stat().st_size}
for asset in s['assets']:
 reuse['assets'][asset['name']]={'file':asset['file'],'instances':[]}
 for ins in asset['instances']:
  m=np.array(ins['world_matrix_mm'],dtype=float);m[:3,3]*=.001
  reuse['assets'][asset['name']]['instances'].append({'key':ins['mesh_key'],'id':ins['id'],'asset_from_part':m.tolist()})
(shared/'reuse.json').write_text(json.dumps(reuse,indent=2)+'\n')
print('Shared files:',len(reuse['parts']),'bytes:',sum(p['bytes'] for p in reuse['parts'].values()))
