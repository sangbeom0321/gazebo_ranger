#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import math


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_ranger')

    world_default = join(pkg_share, 'worlds', 'aws_robomaker_small_warehouse_world.world')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    ranger_xacro = join(pkg_share, 'urdf', 'ranger_mini.xacro')

    robot_description = xacro.process_file(ranger_xacro).toxml()
    
    # Create directory for generated SDF files (in install/share/gazebo_ranger/models/aruco_box/generated/)
    models_dir = join(pkg_share, 'models', 'aruco_box', 'generated')
    os.makedirs(models_dir, exist_ok=True)
    aruco_models = []
    
    # Generate SDF files immediately (before launch actions)
    from xml.etree import ElementTree as ET
    for i in range(10):
        sdf_file = join(models_dir, f'aruco_box_{i}.sdf')
        aruco_models.append(sdf_file)
        
        # Generate SDF file directly
        sdf = ET.Element('sdf', version='1.6')
        model = ET.SubElement(sdf, 'model', name=f'aruco_box_{i}')
        static = ET.SubElement(model, 'static')
        static.text = 'true'
        
        link = ET.SubElement(model, 'link', name='box_link')
        gravity = ET.SubElement(link, 'gravity')
        gravity.text = 'false'
        
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '1.0'
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
        ET.SubElement(box, 'size').text = '0.35 0.35 0.5'
        material_box = ET.SubElement(visual_box, 'material')
        script_box = ET.SubElement(material_box, 'script')
        ET.SubElement(script_box, 'uri').text = 'file://media/materials/scripts/gazebo.material'
        ET.SubElement(script_box, 'name').text = 'Gazebo/Orange'
        
        # Main box collision
        collision_box = ET.SubElement(link, 'collision', name='box_collision')
        geometry_collision = ET.SubElement(collision_box, 'geometry')
        box_collision = ET.SubElement(geometry_collision, 'box')
        ET.SubElement(box_collision, 'size').text = '0.35 0.35 0.5'
        
        # ArUco marker on -X face
        visual_marker_x = ET.SubElement(link, 'visual', name='marker_aruco_neg_x')
        ET.SubElement(visual_marker_x, 'pose').text = '-0.176 0 0 0 0 0'
        geometry_marker_x = ET.SubElement(visual_marker_x, 'geometry')
        box_marker_x = ET.SubElement(geometry_marker_x, 'box')
        ET.SubElement(box_marker_x, 'size').text = '0.003 0.30 0.30'
        material_marker_x = ET.SubElement(visual_marker_x, 'material')
        script_marker_x = ET.SubElement(material_marker_x, 'script')
        uri1_x = ET.SubElement(script_marker_x, 'uri')
        uri1_x.text = 'model://aruco_box/materials/scripts'
        uri2_x = ET.SubElement(script_marker_x, 'uri')
        uri2_x.text = 'model://aruco_box/materials/textures'
        name_x = ET.SubElement(script_marker_x, 'name')
        name_x.text = f'Aruco/Marker{i}'
        
        # ArUco marker on -Z face
        visual_marker_z = ET.SubElement(link, 'visual', name='marker_aruco_neg_z')
        ET.SubElement(visual_marker_z, 'pose').text = '0 0 -0.251 1.5708 0 -1.5708'
        geometry_marker_z = ET.SubElement(visual_marker_z, 'geometry')
        box_marker_z = ET.SubElement(geometry_marker_z, 'box')
        ET.SubElement(box_marker_z, 'size').text = '0.30 0.003 0.30'
        material_marker_z = ET.SubElement(visual_marker_z, 'material')
        script_marker_z = ET.SubElement(material_marker_z, 'script')
        uri1_z = ET.SubElement(script_marker_z, 'uri')
        uri1_z.text = 'model://aruco_box/materials/scripts'
        uri2_z = ET.SubElement(script_marker_z, 'uri')
        uri2_z.text = 'model://aruco_box/materials/textures'
        name_z = ET.SubElement(script_marker_z, 'name')
        name_z.text = f'Aruco/Marker{i}'
        
        # Write to file (without XML declaration to avoid encoding issues)
        tree = ET.ElementTree(sdf)
        # Write without XML declaration to avoid encoding issues with spawn_entity.py
        # Use tostring and manual write (indent will be handled by Gazebo)
        xml_string = ET.tostring(sdf, encoding='unicode', method='xml')
        with open(sdf_file, 'w', encoding='utf-8') as f:
            f.write(xml_string)

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(pkg_share, 'models')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': 'true',
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    spawn_ranger = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ranger_mini',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '-1.5708'
        ]
    )

    # Spawn ArUco boxes with specified positions, all facing (0,0)
    spawn_aruco_boxes = []
    # Positions: (x, y) - z will be 0.25 (half box height)
    positions_xy = [
        (0.0, 3.0),      # Box 0
        (1.9, 3.0),      # Box 1
        (1.9, -1.5),     # Box 2
        (4.6, -1.5),     # Box 3
        (4.6, -3.75),    # Box 4
        (4.6, -6.0),     # Box 5
        (1.45, -6.0),    # Box 6
        (-1.7, -6.0),    # Box 7
        (-1.9, 3.0),     # Box 8
        (-1.7, -1.5),    # Box 9
    ]
    
    # Calculate yaw to face center point (1.495, -2.425) from each position
    center_x = 1.495
    center_y = -2.425
    for i in range(10):
        x, y = positions_xy[i]
        z = 0.25  # Half box height
        # Calculate yaw to face center point (center_x, center_y)
        # Direction vector from box position (x,y) to center: (center_x - x, center_y - y)
        # Add π to reverse direction if boxes are facing away
        dx = center_x - x
        dy = center_y - y
        yaw = math.atan2(dy, dx) + math.pi
        spawn_aruco_boxes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'aruco_box_{i}',
                    '-file', aruco_models[i],
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(yaw)
                ],
                output='screen'
            )
        )
    
    # Load controllers
    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller']
    )

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller']
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster']
    )
    
    # Ranger controller key - converts cmd_vel to controller commands
    ranger_controller_key = Node(
        package='gazebo_ranger',
        executable='ranger_controller_key.py',
        name='commander',
        output='screen'
    )

    # You have to turn on this node, when you want to control
    ranger_controller = Node(
        package='gazebo_ranger',
        executable='ranger_controller.py',
        name='commander',
        output='screen'
    )

    utm_pub = Node(
        package='gazebo_ranger',
        executable='utm_publisher.py',
        name='commander',
        output='screen'
    )

    fake_local_path = Node(
        package='gazebo_ranger',
        executable='fake_local_path.py',
        name='commander',
        output='screen'
    )

    state_machine = Node(
        package='gazebo_ranger',
        executable='state_machine',
        name='commander',
        output='screen'
    )

    rviz_config = join(pkg_share, 'rviz', 'rviz_setup.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # PointCloud to LaserScan converter (Z축 필터링만 사용)
    laser_scan_node = Node(
        package='gazebo_ranger',
        executable='laser_scan',
        name='laser_scan',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pcd_topic': '/points',
            'scan_topic': '/scan',
            'clipping_minz': -0.2,
            'clipping_maxz': 0.4,
            'scan_frame_id': 'ouster_base_link',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,  # 1도
            'range_min': 0.5,
            'range_max': 200.0
        }],
        output='screen'
    )

    # Camera pose is now set in the world file (aws_robomaker_small_warehouse_world.world)
    # No need for additional camera pose setting script

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_default),
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_ranger,
        *spawn_aruco_boxes,       # Spawn all boxes (SDF files already generated)
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster,
        ranger_controller_key,
        # ranger_controller,
        utm_pub,
        fake_local_path,
        state_machine,
        rviz_node,
        laser_scan_node
    ])


