import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'gazebo_viz'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Path to the xacro file
    pkg_path = os.path.join(get_package_share_directory('gazebo_viz'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Generate robot_description from the xacro file
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config}

    load_controllers = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[params],
    )

    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    # depth_to_scan = Node(
    #     package='depthimage_to_laserscan',
    #     executable='depthimage_to_laserscan_node',
    #     name='depthimage_to_laserscan',
    #     output='screen',
    #     remappings=[
    #         ('depth', '/camera/depth/image_raw'),
    #         ('depth_camera_info', '/camera/depth/camera_info'),
    #         ('scan', '/scan_camera'),
    #     ],
    #     parameters=[{
    #         'output_frame_id': 'base_footprint',
    #         'range_min': 0.1,
    #         'range_max': 100.0,
    #         'scan_height': 1,
    #         'angle_min': -0.785*2,
    #         'angle_max': 0.785*2,
    #         'use_sim_time': True,
    #     }]
    # )

    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'ldlidar_base']
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        load_controllers,
        spawn_diff_drive_controller,
        joint_broad_spawner,
        # depth_to_scan,
        static_tf_base_to_lidar
    ])
