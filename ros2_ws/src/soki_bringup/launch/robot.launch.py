import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_dir = get_package_share_directory('soki_description')
    bringup_dir = get_package_share_directory('soki_bringup')

    xacro_file = os.path.join(description_dir, 'urdf', 'soki_robo.urdf.xacro')
    controllers_yaml = os.path.join(bringup_dir, 'config', 'controllers.yaml')
    ekf_yaml = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    robot_description = Command(['xacro ', xacro_file])

    # 1. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # 2. ros2_control_node (controller_manager)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # 3. Spawners (delayed to wait for controller_manager)
    def create_spawner(controller_name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name],
            output='screen',
        )

    spawner_jsb = create_spawner('joint_state_broadcaster')
    spawner_ddc = create_spawner('diff_drive_controller')
    spawner_imu = create_spawner('imu_sensor_broadcaster')

    # Delay spawners until controller_manager is ready
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[spawner_jsb, spawner_ddc, spawner_imu],
    )

    # 4. robot_localization EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml],
        output='screen',
    )

    # 5. RPLiDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',    # PLACEHOLDER: LiDAR USB port
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        ekf_node,
        rplidar_node,
    ])
