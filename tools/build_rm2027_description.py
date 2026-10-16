#!/usr/bin/env python3
"""Build the full-color RM2027 scene from audited split assets (offline tool)."""
import argparse,copy,json,math,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

from description_layout import mesh_path, read_system, write_system

NS='http://www.ros.org/wiki/xacro';ET.register_namespace('xacro',NS)
def x(tag):return '{'+NS+'}'+tag
def numbers(v):return ' '.join(f'{a:.12g}' for a in v)
def pose(R=None,p=None):
 m=np.eye(4)
 if R is not None:m[:3,:3]=R
 if p is not None:m[:3,3]=p
 return m

def station_mount_datum(part):
 """Selected A face (71, definition 31), expressed in the station frame."""
 station_rotation=np.column_stack(([0,0,-1],[1,0,0],[0,-1,0]))
 triangles=part['triangles'][part['triangle_face_indices']==71]
 if not len(triangles):raise ValueError('Selected launcher mounting face 71 is missing')
 points=part['vertices'][triangles]@station_rotation
 flat=points.reshape(-1,3);center=(flat.min(0)+flat.max(0))/2
 normal=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0]).sum(0)
 normal/=np.linalg.norm(normal)
 if normal[2]<0:normal=-normal
 if np.max(np.abs((flat-center)@normal))>1e-6:raise ValueError('Mounting face is not planar')
 forward=np.array([1.,0.,0.]);forward-=normal*np.dot(forward,normal);forward/=np.linalg.norm(forward)
 return pose(np.column_stack((forward,np.cross(normal,forward),normal)),center)

def station_gate_open_offset(part):
 """CAD-plane contact: moving bracket 16284 against fixed underside 4541.

 Exact BRep overlap and OUT/ON/IN checks are recorded in gate_contact.json.
 Recompute the along-guide displacement from the source faces on rebuild.
 """
 rotation=np.column_stack(([0,0,-1],[1,0,0],[0,-1,0]))
 def plane(face):
  t=part['triangles'][part['triangle_face_indices']==face]
  if not len(t):raise ValueError(f'Missing gate contact face {face}')
  v=part['vertices'][t]@rotation;cross=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0])
  center=np.average(v.mean(1),axis=0,weights=np.linalg.norm(cross,axis=1))
  normal=cross.sum(0);normal/=np.linalg.norm(normal)
  if np.max(np.abs((v-center)@normal))>1e-6:raise ValueError('Gate contact face is not planar')
  return center,normal
 moving,mn=plane(16284);fixed,fn=plane(4541)
 axis=np.array([-math.sin(math.pi/12),0,math.cos(math.pi/12)])
 if mn@fn>-.999 or mn@axis<.999:raise ValueError('Gate contact normals changed')
 return float((fixed-moving)@fn/(axis@fn))

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('assets',type=Path);parser.add_argument('--launcher-source',type=Path,required=True);args=parser.parse_args()
 package=Path(__file__).resolve().parents[1];assets=args.assets.resolve();dest=package/'meshes';dest.mkdir(exist_ok=True)
 alignment=json.load(open(package/'metadata/field_alignment_2027.json'));A=np.array(alignment['field_from_cad'])
 manifest=json.load(open(assets/'split/split_manifest.json'));pieces={a['name']:a for a in manifest['assets']};rows=json.load(open(assets/'assembly_inventory.json'))['nodes'];parts={p['definition_label']:p for p in json.load(open(assets/'mesh_manifest.json'))['parts']}
 display_source=assets/'optimized' if (assets/'optimized/validation.json').is_file() else assets/'split'
 for a in pieces.values():
  target=dest/mesh_path(a['file']);target.parent.mkdir(exist_ok=True);shutil.copy2(display_source/a['file'],target)
 root=ET.Element('robot',name='dart_system');root.append(ET.Comment('RM2027 generated scene. field_link: red-side right corner, x forward, y left, z vertical.'))
 source=read_system(args.launcher_source)
 for e in source:
  if e.tag=='material' and e.get('name') in ['aluminum','camera']:root.append(copy.deepcopy(e))
 ET.SubElement(root,x('arg'),name='team',default='red');ET.SubElement(root,x('property'),name='team',value='$(arg team)')
 frames={'field_link':np.eye(4)};audit={'datum_status':'CAD-derived visualization frames; not surveyed mechanical datums','gate':{},'launcher':{},'targets':{},'visuals':[]}
 def origin(element,m):ET.SubElement(element,'origin',xyz=numbers(m[:3,3]),rpy=numbers(Rotation.from_matrix(m[:3,:3]).as_euler('xyz')))
 def link(name,parent=None,world=None,joint=None,kind='fixed',axis=None,limits=None):
  element=ET.SubElement(root,'link',name=name)
  if parent is not None:
   j=ET.SubElement(root,'joint',name=joint or parent+'_to_'+name,type=kind);ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name);origin(j,np.linalg.inv(frames[parent])@world)
   if axis is not None:ET.SubElement(j,'axis',xyz=numbers(axis))
   if limits is not None:ET.SubElement(j,'limit',lower=str(limits[0]),upper=str(limits[1]),effort='1',velocity='1')
   frames[name]=world
  return element
 def visual(element,asset):
  a=pieces[asset];m=A@pose(p=a['local_to_cad_translation_m']);relative=np.linalg.inv(frames[element.get('name')])@m
  vis=ET.SubElement(element,'visual');origin(vis,relative);geo=ET.SubElement(vis,'geometry');ET.SubElement(geo,'mesh',filename='package://dart_description/meshes/'+mesh_path(a['file']).as_posix(),scale='1 1 1')
  audit['visuals'].append({'link':element.get('name'),'asset':asset,'field_from_asset_zero':m.tolist()})
 def cad(i):
  m=np.array(rows[i]['world_matrix_mm']);m[:3,3]*=.001;return m
 def mesh(key):return dict(np.load(assets/parts[key]['file']))
 def target_data(solid_key,green_faces,shell_key,armor_face):
  a=mesh(solid_key);gt=a['vertices'][a['triangles'][np.isin(a['triangle_face_indices'],green_faces)]];q=gt.reshape(-1,3);green=(q.min(0)+q.max(0))/2
  gn=np.cross(gt[:,1]-gt[:,0],gt[:,2]-gt[:,0]).sum(0);gn/=np.linalg.norm(gn);gy=np.cross([0.,1.,0.],gn);gy/=np.linalg.norm(gy);greenR=np.column_stack((gn,gy,np.cross(gn,gy)))
  b=mesh(shell_key);t=b['vertices'][b['triangles'][b['triangle_face_indices']==armor_face]];q=t.reshape(-1,3);center=(q.min(0)+q.max(0))/2;n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]).sum(0);n/=np.linalg.norm(n)
  if n[1]<0:n=-n
  up=np.array([0.,1.,0.]);left=np.cross(up,n);left/=np.linalg.norm(left);R=np.column_stack((n,left,np.cross(n,left)))
  return green,pose(R,center),greenR
 floor=link('field_link');visual(floor,'field_static')
 base_rotation=np.array([[0,1,0],[0,0,1],[1,0,0]],float) # forward CAD Z, left CAD X, up CAD Y
 for side,idx in [('red',106),('blue',109)]:
  W=A@cad(idx);B=W@pose(base_rotation)
  body=link(side+'_base_link','field_link',B);visual(body,'base_'+side+'_body')
  rail_world=W@pose(base_rotation,[0,1.0423,-.0448]);rail=link(side+'_rail_link',side+'_base_link',rail_world);visual(rail,'base_'+side+'_rail')
  jname=side+'_base_target_slide_joint'
  target=link(side+'_base_target_link',side+'_rail_link',rail_world,joint=jname,kind='prismatic',axis=[0,1,0],limits=[-.28,.28]);visual(target,'base_'+side+'_target')
  green,armor,greenR=target_data('0:1:1:64',[44,46],'0:1:1:64',140)
  link(side+'_base_green_light_link',side+'_base_target_link',W@pose(greenR,green))
  link(side+'_base_armor_link',side+'_base_target_link',W@armor)
  audit['targets'][side+'_base']={'green_center_definition_m':green.tolist(),'green_rotation_definition':greenR.tolist(),'green_normal_method':'area-weighted normal of CAD emitting faces','green_center_method':'bounding center of emitted green CAD faces 44 and 46','armor_definition':'0:1:1:64','armor_face':140,'armor_frame_definition':armor.tolist(),'armor_surface':'outer central contact face; local +x points outward','slide_axis':'base local +y, parallel to field width','slide_limit_m':[-.28,.28],'rule_reference':'RM2027 V1.0.0 printed page 111'}
 for side,idx,asset in [('red',21,'outpost_020'),('blue',18,'outpost_017')]:
  W=A@cad(idx);R=np.column_stack(([1,0,0],[0,0,-1],[0,1,0]));B=W@pose(R)
  body=link(side+'_outpost_link','field_link',B);visual(body,asset+'_body')
  green,armor,greenR=target_data('0:1:1:16',[7077,7079],'0:1:1:16',17250)
  target=link(side+'_outpost_target_link',side+'_outpost_link',W@pose(R,green));visual(target,asset+'_target')
  link(side+'_outpost_green_light_link',side+'_outpost_target_link',W@pose(greenR,green));link(side+'_outpost_armor_link',side+'_outpost_target_link',W@armor)
  audit['targets'][side+'_outpost']={'green_center_definition_m':green.tolist(),'green_rotation_definition':greenR.tolist(),'green_normal_method':'area-weighted normal of CAD emitting faces','armor_definition':'0:1:1:16','armor_face':17250,'armor_frame_definition':armor.tolist(),'armor_surface':'outer central contact face; local +x points outward','motion':'fixed; middle armor rotor remains visual-only'}
 gate_offset=station_gate_open_offset(mesh('0:1:1:31'))
 gate_axis=np.array([-math.sin(math.pi/12),0,math.cos(math.pi/12)])
 audit['gate']={'zero':'lower closed pose, 1.2 m below the CAD-derived upper contact',
  'positive':'opening upwards along inclined guide','travel_m':1.2,'travel_status':'user-confirmed stroke',
  'cad_reference':'source CAD pose is not an endpoint','open_offset_from_cad_along_guide_m':gate_offset,
  'axis_source':'CAD station guide slope 15 deg in definition 0:1:1:31',
  'closed_origin_station_m':((gate_offset-1.2)*gate_axis).tolist(),'opening_axis_station':gate_axis.tolist(),
  'contact':{'definition':'0:1:1:31','moving_face':16284,'fixed_face':4541,'evidence':'metadata/gate_contact.json'}}
 mount_local=station_mount_datum(mesh('0:1:1:31'))
 audit['launcher']={'mount':'user-selected A: lower chamber face 71 of CAD definition 0:1:1:31',
  'origin_method':'full face axis-aligned bounding center in station coordinates',
  'axes':'z = upward face normal; x = station forward projected onto face; y = z cross x',
  'station_from_mount':mount_local.tolist(),
  'status':'selected CAD mounting datum; actual rig installation offsets remain adjustable'}
 for side,idx in [('red',39),('blue',57)]:
  W=A@cad(idx);R=np.column_stack(([0,0,-1],[1,0,0],[0,-1,0]));B=W@pose(R)
  station=link(side+'_dart_station_link','field_link',B);visual(station,'dart_station_'+side+'_fixed')
  direction=R.T@np.array([0,-math.cos(math.pi/12),math.sin(math.pi/12)])
  # CAD pose is not an end stop. The upper endpoint is the bracket/frame
  # contact; subtract the confirmed stroke to establish the lower endpoint.
  closed=B@pose(p=(gate_offset-1.2)*direction)
  opened=B@pose(p=gate_offset*direction)
  gate=link(side+'_dart_gate_link',side+'_dart_station_link',closed,joint=side+'_dart_gate_joint',kind='prismatic',axis=direction,limits=[0,'1.2']);visual(gate,'dart_station_'+side+'_gate')
  cad_visual=np.array(audit['visuals'][-1]['field_from_asset_zero'])
  gate.find('visual').remove(gate.find('visual/origin'))
  origin(gate.find('visual'),np.linalg.inv(B)@cad_visual)
  audit['visuals'][-1]['field_from_asset_cad_reference']=cad_visual.tolist()
  audit['visuals'][-1]['field_from_asset_open']=(opened@np.linalg.inv(B)@cad_visual).tolist()
  audit['visuals'][-1]['field_from_asset_zero']=(closed@np.linalg.inv(B)@cad_visual).tolist()
  # Both stations instantiate definition 31, so share the same local datum.
  mount=B@mount_local
  link(side+'_launcher_mount_link',side+'_dart_station_link',mount)
 # Unprefixed target frames select the opponent for the package geometry API.
 for side in ['red','blue']:
  cond=ET.SubElement(root,x('if'),value="${team == '"+side+"'}");opponent='blue' if side=='red' else 'red'
  for name,parent in [('base_link',opponent+'_base_link'),('rail_origin_link',opponent+'_rail_link'),('dart_detection_module_link',opponent+'_base_target_link'),('green_light_link',opponent+'_base_green_light_link'),('armor_link',opponent+'_base_armor_link'),('outpost_target_link',opponent+'_outpost_target_link'),('outpost_green_light_link',opponent+'_outpost_green_light_link'),('outpost_armor_link',opponent+'_outpost_armor_link'),('dart_pedestal_link',side+'_launcher_mount_link')]:
   e=ET.SubElement(cond,'link',name=name);j=ET.SubElement(cond,'joint',name='alias_'+name,type='fixed');ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name);ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0')
   if name=='armor_link':
    collision=ET.SubElement(e,'collision');ET.SubElement(collision,'origin',xyz="-0.005 0 0",rpy='0 ${pi / 2} 0');geometry=ET.SubElement(collision,'geometry');ET.SubElement(geometry,'box',size='0.1 0.1 0.01')
 j=ET.SubElement(root,'joint',name='dart_pedestal_to_base',type='fixed')
 ET.SubElement(j,'parent',link='dart_pedestal_link');ET.SubElement(j,'child',link='dart_base_link')
 mount_joint=source.find("joint[@name='dart_pedestal_to_base']")
 if mount_joint is not None:j.append(copy.deepcopy(mount_joint.find('origin')))
 else:ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0')
 # Reuse only the user's launcher and camera internals; replace its mounting origin.
 keep=False
 for e in source:
  if e.tag=='link' and e.get('name')=='dart_base_link':keep=True
  if keep:
   if e.tag=='joint' and e.get('name')=='dart_pedestal_to_base':continue
   e=copy.deepcopy(e)
   root.append(e)
 write_system(root,package)
 (package/'metadata/rm2027_mechanisms.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
 if (display_source/'reused/reuse.json').is_file():
  from reuse_display_meshes import install_reused
  install_reused(display_source,package)
 print('Generated',len(frames),'CAD frames and',len(audit['visuals']),'colored visuals')

if __name__=='__main__':main()
