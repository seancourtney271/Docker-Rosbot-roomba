from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('roombot')

    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    # slam_config = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(pkg, 'config', 'nav2_params.yaml')

    urdf_file = os.path.join(
        get_package_share_directory('roombot'),
        'urdf',
        'roombot_physical.urdf'
    )
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Encoder odom node
        Node(
            package='roombot',
            executable='encoder_odom_node',
            name='encoder_odom_node',
            parameters=[{'publish_tf': False}]  # important!
        ),

        # BNO055 IMU Driver
        Node(
            package='bno055',
            executable='bno055',
            name='bno055_node',
            output='screen',
            parameters=[
                {'connection_type': 'i2c'},
                {'i2c_bus': 1},
                {'i2c_address': 40},  # 40 decimal == 0x28; use 41 for 0x29
                # Optional:
                # {'frame_id': 'bno055'},
                # {'ros_topic_prefix': 'bno055/'},
            ],
        ),

        # RPLidar Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', 
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': True
            }],
            remappings=[('/scan', 'scan')]
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # SLAM Toolbox
        # Node(
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',
        #     parameters=[slam_config],
        #     output='screen'
        # ),

        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Nav2 bringup (mapping mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'slam_launch.py'
                )
            ),
            launch_arguments={
                'slam': 'True',
                'use_sim_time': 'False',
                'params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            }.items()
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": 8765, "address": "0.0.0.0"}]
        ),

    ])
