import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('soki_navigation')
    mapper_params = os.path.join(nav_dir, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_params],
        output='screen',
    )

    return LaunchDescription([
        slam_toolbox_node,
    ])
