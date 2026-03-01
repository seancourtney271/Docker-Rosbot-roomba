
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from pathlib import Path

URDF_PATH = os.path.join(
    os.getenv('HOME'), 'ws_roombot', 'src', 'roombot', 'urdf', 'roombot.urdf'
)
RVIZ_CONFIG_PATH = os.path.join(
    os.getenv('HOME'), 'ws_roombot', 'src', 'roombot', 'rviz', 'view_robot.rviz'
)

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Path(URDF_PATH).read_text()}]
        ),

        # Joint State Publisher (for wheel joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
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
            remappings=[('/imu/data', 'imu/data')],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', RVIZ_CONFIG_PATH]
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": 8765, "address": "0.0.0.0"}]
        ),
    ])
