"""Install validated definition-local meshes and expand visuals without adding TFs."""
import copy
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from description_layout import mesh_path


def matrix(origin):
    result=np.eye(4)
    if origin is not None:
        result[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()))
        result[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
    return result


def install_reused(optimized,package):
    optimized,package=Path(optimized),Path(package)
    shared=optimized/'reused'
    reuse=json.loads((shared/'reuse.json').read_text())
    by_file={a['file']:a for a in reuse['assets'].values()}
    target=package/'meshes/shared';target.mkdir(exist_ok=True)
    for part in reuse['parts'].values():shutil.copy2(shared/part['file'],target/part['file'])
    for path in (package/'urdf/modules').glob('*.xacro'):
        root=ET.parse(path).getroot();changed=False
        for link in root.iter('link'):
            for visual in list(link.findall('visual')):
                mesh=visual.find('geometry/mesh')
                if mesh is None:continue
                asset=by_file.get(Path(mesh.get('filename')).name)
                if asset is None:continue
                old=matrix(visual.find('origin'));index=list(link).index(visual)
                link.remove(visual)
                for n,ins in enumerate(asset['instances']):
                    m=old@np.array(ins['asset_from_part'])
                    v=copy.deepcopy(visual);v.set('name',f"cad_{ins['id']}_{ins['key'].replace(':','_')}")
                    o=v.find('origin')
                    if o is None:o=ET.SubElement(v,'origin')
                    o.set('xyz',' '.join(f'{x:.15g}' for x in m[:3,3]))
                    o.set('rpy',' '.join(f'{x:.15g}' for x in Rotation.from_matrix(m[:3,:3]).as_euler('xyz')))
                    v.find('geometry/mesh').set('filename','package://dart_description/meshes/shared/'+reuse['parts'][ins['key']]['file'])
                    link.insert(index+n,v)
                changed=True
        if changed:
            ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    audit_path=package/'metadata/rm2027_mechanisms.json'
    audit=json.loads(audit_path.read_text());visuals=[]
    for item in audit['visuals']:
        if 'mesh_key' in item:visuals.append(item);continue
        for ins in reuse['assets'][item['asset']]['instances']:
            entry=copy.deepcopy(item)
            entry.update(mesh_key=ins['key'],visual_name=f"cad_{ins['id']}_{ins['key'].replace(':','_')}",mesh_file='shared/'+reuse['parts'][ins['key']]['file'])
            for key in list(entry):
                if key.startswith('field_from_asset_'):entry[key]=(np.array(entry[key])@np.array(ins['asset_from_part'])).tolist()
            visuals.append(entry)
    audit['visuals']=visuals
    audit_path.write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    (package/'metadata/mesh_reuse.json').write_text(json.dumps(reuse,indent=2)+'\n')
    # Remove only replaced, unreferenced legacy assets, never unrelated user files.
    references={Path(m.get('filename')).name for p in (package/'urdf').rglob('*.xacro') for m in ET.parse(p).getroot().iter('mesh')}
    for name in by_file:
        path=package/'meshes'/mesh_path(name)
        if name not in references and path.exists():path.unlink()
    return reuse


def validate_reused(optimized,lib):
    """Compare every vertex, normal, material triangle against the baked export."""
    optimized=Path(optimized);reuse=json.loads((optimized/'reused/reuse.json').read_text())
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    def read(path):
        root=ET.parse(path).getroot()
        arrays=root.findall('.//c:geometry/c:mesh/c:source/c:float_array',ns)
        vertices,normals=[np.fromstring(a.text,sep=' ').reshape(-1,3) for a in arrays]
        colors={e.get('id').removesuffix('fx'):tuple(np.fromstring(e.find('.//c:diffuse/c:color',ns).text,sep=' ')) for e in root.findall('.//c:effect',ns)}
        triangles={colors[t.get('material')]:np.fromstring(t.find('c:p',ns).text or '',sep=' ',dtype=np.int64).reshape(-1,3) for t in root.findall('.//c:triangles',ns)}
        return vertices,normals,triangles
    cache={}
    for key,p in reuse['parts'].items():
        path=optimized/'reused'/p['file'];scene=lib.aiImportFile(str(path).encode(),0)
        if not scene:raise ValueError(f'Assimp failed: {path}')
        lib.aiReleaseImport(scene);cache[key]=read(path)
    worst_position=worst_normal=0.
    for asset in reuse['assets'].values():
        old_v,old_n,old_t=read(optimized/asset['file']);offset=0;groups={}
        for ins in asset['instances']:
            v,n,tri=cache[ins['key']];m=np.array(ins['asset_from_part']);length=len(v)
            worst_position=max(worst_position,float(np.abs(v@m[:3,:3].T+m[:3,3]-old_v[offset:offset+length]).max()))
            worst_normal=max(worst_normal,float(np.abs(n@m[:3,:3].T-old_n[offset:offset+length]).max()))
            for color,t in tri.items():groups.setdefault(color,[]).append(t+offset)
            offset+=length
        assert offset==len(old_v)
        for color,t in old_t.items():np.testing.assert_array_equal(t,np.concatenate(groups[color]))
        assert set(groups)==set(old_t)
    assert worst_position<2e-6, worst_position
    assert worst_normal<2e-7, worst_normal
    return {'unique_files':len(cache),'visual_instances':sum(len(a['instances']) for a in reuse['assets'].values()),
            'unique_triangles':sum(p['triangles'] for p in reuse['parts'].values()),
            'scene_triangles':sum(reuse['parts'][i['key']]['triangles'] for a in reuse['assets'].values() for i in a['instances']),
            'shared_bytes':sum(p['bytes'] for p in reuse['parts'].values()),
            'previous_bytes':sum((optimized/a['file']).stat().st_size for a in reuse['assets'].values()),
            'max_position_rounding_error_m':worst_position,'max_normal_rounding_error':worst_normal,
            'material_triangle_indices_identical':True,'assimp_imports_passed':len(cache)}
