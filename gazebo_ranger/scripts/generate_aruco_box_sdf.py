#!/usr/bin/env python3
"""
Generate ArUco box SDF with specified marker ID
Usage: python3 generate_aruco_box_sdf.py <marker_id> <output_file>
"""

import sys
import os
from xml.etree import ElementTree as ET

def generate_aruco_box_sdf(marker_id, output_file):
    """Generate ArUco box SDF with specified marker ID"""
    
    # Create SDF root
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=f'aruco_box_{marker_id}')
    
    # Add comment
    comment = ET.Comment('Pose will be set via spawn_entity.py arguments')
    model.append(comment)
    static_comment = ET.Comment('static=true: 중력 영향 없이 고정된 위치에 유지')
    model.append(static_comment)
    
    # Set static
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    
    # Create link
    link = ET.SubElement(model, 'link', name='box_link')
    gravity = ET.SubElement(link, 'gravity')
    gravity.text = 'false'
    
    # Inertial
    inertial = ET.SubElement(link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = '1.0'
    inertia = ET.SubElement(inertial, 'inertia')
    ET.SubElement(inertia, 'ixx').text = '0.166667'
    ET.SubElement(inertia, 'ixy').text = '0'
    ET.SubElement(inertia, 'ixz').text = '0'
    ET.SubElement(inertia, 'iyy').text = '0.166667'
    ET.SubElement(inertia, 'iyz').text = '0'
    ET.SubElement(inertia, 'izz').text = '0.083333'
    
    # Main box visual
    visual_box = ET.SubElement(link, 'visual', name='box_visual')
    geometry_box = ET.SubElement(visual_box, 'geometry')
    box = ET.SubElement(geometry_box, 'box')
    size_box = ET.SubElement(box, 'size')
    size_box.text = '0.35 0.35 0.5'
    material_box = ET.SubElement(visual_box, 'material')
    script_box = ET.SubElement(material_box, 'script')
    ET.SubElement(script_box, 'uri').text = 'file://media/materials/scripts/gazebo.material'
    ET.SubElement(script_box, 'name').text = 'Gazebo/Orange'
    
    # Main box collision
    collision_box = ET.SubElement(link, 'collision', name='box_collision')
    geometry_collision = ET.SubElement(collision_box, 'geometry')
    box_collision = ET.SubElement(geometry_collision, 'box')
    size_collision = ET.SubElement(box_collision, 'size')
    size_collision.text = '0.35 0.35 0.5'
    
    # ArUco marker on -X face
    visual_marker_x = ET.SubElement(link, 'visual', name='marker_aruco_neg_x')
    pose_marker_x = ET.SubElement(visual_marker_x, 'pose')
    pose_marker_x.text = '-0.176 0 0 0 0 0'
    geometry_marker_x = ET.SubElement(visual_marker_x, 'geometry')
    box_marker_x = ET.SubElement(geometry_marker_x, 'box')
    size_marker_x = ET.SubElement(box_marker_x, 'size')
    size_marker_x.text = '0.003 0.30 0.30'
    material_marker_x = ET.SubElement(visual_marker_x, 'material')
    script_marker_x = ET.SubElement(material_marker_x, 'script')
    uri1_x = ET.SubElement(script_marker_x, 'uri')
    uri1_x.text = 'model://aruco_box/materials/scripts'
    uri2_x = ET.SubElement(script_marker_x, 'uri')
    uri2_x.text = 'model://aruco_box/materials/textures'
    name_x = ET.SubElement(script_marker_x, 'name')
    name_x.text = f'Aruco/Marker{marker_id}'
    
    # ArUco marker on -Z face
    visual_marker_z = ET.SubElement(link, 'visual', name='marker_aruco_neg_z')
    pose_marker_z = ET.SubElement(visual_marker_z, 'pose')
    pose_marker_z.text = '0 0 -0.251 1.5708 0 -1.5708'
    geometry_marker_z = ET.SubElement(visual_marker_z, 'geometry')
    box_marker_z = ET.SubElement(geometry_marker_z, 'box')
    size_marker_z = ET.SubElement(box_marker_z, 'size')
    size_marker_z.text = '0.30 0.003 0.30'
    material_marker_z = ET.SubElement(visual_marker_z, 'material')
    script_marker_z = ET.SubElement(material_marker_z, 'script')
    uri1_z = ET.SubElement(script_marker_z, 'uri')
    uri1_z.text = 'model://aruco_box/materials/scripts'
    uri2_z = ET.SubElement(script_marker_z, 'uri')
    uri2_z.text = 'model://aruco_box/materials/textures'
    name_z = ET.SubElement(script_marker_z, 'name')
    name_z.text = f'Aruco/Marker{marker_id}'
    
    # Write to file
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    
    print(f"Generated ArUco box SDF with marker ID {marker_id} to {output_file}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 generate_aruco_box_sdf.py <marker_id> <output_file>")
        sys.exit(1)
    
    marker_id = int(sys.argv[1])
    output_file = sys.argv[2]
    
    generate_aruco_box_sdf(marker_id, output_file)
