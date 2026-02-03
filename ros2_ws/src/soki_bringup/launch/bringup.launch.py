import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('soki_navigation')
    bringup_dir = get_package_share_directory('soki_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    mapper_params = os.path.join(nav_dir, 'config', 'mapper_params_online_async.yaml')

    # --- Launch arguments ---
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Run in SLAM mode (true) or localization mode (false)',
    )

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map yaml (required when slam:=false)',
    )

    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')

    # --- Robot bringup (common) ---
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
    )

    # --- SLAM mode: slam_toolbox ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_params],
        output='screen',
        condition=IfCondition(slam),
    )

    # --- Localization mode: nav2 bringup (AMCL + nav2 stack) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items(),
        condition=UnlessCondition(slam),
    )

    return LaunchDescription([
        slam_arg,
        map_arg,
        robot_launch,
        slam_toolbox_node,
        nav2_launch,
    ])
