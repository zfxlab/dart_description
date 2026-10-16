"""Shared module and mesh layout for the offline description generator."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

NS = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', NS)
MODULES = ('field', 'bases', 'outposts', 'dart_stations', 'targets', 'launcher', 'stereo_camera')


def mesh_path(filename):
    name = Path(filename).name
    if name.startswith('part_'):
        folder = 'shared'
    elif name.startswith('field_'):
        folder = 'field'
    elif name.startswith('base_'):
        folder = 'bases'
    elif name.startswith('outpost_'):
        folder = 'outposts'
    elif name.startswith('dart_station_'):
        folder = 'dart_stations'
    elif name.startswith('camera_mount_'):
        folder = 'stereo_camera'
    else:
        folder = 'launcher'
    return Path(folder) / name


def read_system(path):
    path = Path(path)
    root = ET.parse(path).getroot()
    result = ET.Element('robot', name=root.get('name', 'dart_system'))
    for element in root:
        if element.tag == '{' + NS + '}include':
            result.extend(read_system(path.parent / element.get('filename')))
        else:
            result.append(copy.deepcopy(element))
    return result


def write_system(root, package):
    """Write a small entry point and independent functional xacro includes."""
    entry = ET.Element('robot', name='dart_system')
    modules = {name: ET.Element('robot', name=name) for name in MODULES}
    for element in root:
        if element.tag in ('link', 'joint'):
            name = element.get('name', '')
            if element.tag == 'joint':
                name = element.find('child').get('link')
            if element.tag == 'joint' and element.get('name') == 'dart_pedestal_to_base':
                module = 'targets'
            elif name == 'field_link':
                module = 'field'
            elif name.startswith(('red_', 'blue_')):
                module = 'outposts' if 'outpost' in name else 'dart_stations' if any(s in name for s in ('dart_station', 'dart_gate', 'launcher_mount')) else 'bases'
            elif 'camera' in name:
                module = 'stereo_camera'
            else:
                module = 'launcher'
            modules[module].append(element)
        elif element.tag == '{' + NS + '}if':
            modules['targets'].append(element)
        else:
            entry.append(element)
    materials = ET.Element('robot', name='materials')
    for element in list(entry):
        if element.tag == 'material':
            entry.remove(element)
            materials.append(element)
    entry.insert(0, ET.Element('{'+NS+'}include', filename='modules/materials.xacro'))
    runtime = ET.Element('robot', name='dart_robot')
    for name in ('materials', 'launcher', 'stereo_camera'):
        ET.SubElement(runtime, '{'+NS+'}include', filename='modules/'+name+'.xacro')
    for name in MODULES:
        ET.SubElement(entry, '{' + NS + '}include', filename='modules/' + name + '.xacro')
    for tree in [entry, *modules.values()]:
        for mesh in tree.iter('mesh'):
            mesh.set('filename', 'package://dart_description/meshes/' + mesh_path(mesh.get('filename')).as_posix())
    directory = package / 'urdf/modules'
    directory.mkdir(exist_ok=True)
    modules['materials'] = materials
    ET.indent(runtime)
    ET.ElementTree(runtime).write(package / 'urdf/dart_robot.urdf.xacro', encoding='utf-8', xml_declaration=True)
    for name, tree in modules.items():
        ET.indent(tree)
        ET.ElementTree(tree).write(directory / (name + '.xacro'), encoding='utf-8', xml_declaration=True)
    ET.indent(entry)
    ET.ElementTree(entry).write(package / 'urdf/dart_system.urdf.xacro', encoding='utf-8', xml_declaration=True)
