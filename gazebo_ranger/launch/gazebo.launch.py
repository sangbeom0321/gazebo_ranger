#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_ranger')

    world_default = join(pkg_share, 'worlds', 'empty.sdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    ranger_xacro = join(pkg_share, 'urdf', 'ranger_mini.xacro')
    aruco_model = join(pkg_share, 'models', 'aruco_box', 'model.sdf')

    robot_description = xacro.process_file(ranger_xacro).toxml()

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
            '-z', '0.5'
        ]
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

    # Camera pose is now set in the world file (empty.sdf)
    # No need for additional camera pose setting script

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_default),
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_ranger,
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


