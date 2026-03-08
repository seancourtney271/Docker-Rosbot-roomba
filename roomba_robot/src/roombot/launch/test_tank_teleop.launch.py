
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    joy_params = LaunchConfiguration('joy_params')
    teleop_params = LaunchConfiguration('teleop_params')
    driver_params = LaunchConfiguration('driver_params')

    return LaunchDescription([
        DeclareLaunchArgument('joy_params', default_value=''),
        DeclareLaunchArgument('teleop_params', default_value=os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'roombot', 'config', 'tank_teleop.yaml')),
        DeclareLaunchArgument('driver_params', default_value=os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'roombot', 'config', 'l298n_pins.yaml')),

        # Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params] if str(joy_params) else []
            # For most setups, no params needed
        ),

        # Teleop remaps / parameters
        Node(
            package='roombot',
            executable='teleop_ps4',
            name='teleop_ps4',
            output='screen',
            parameters=[teleop_params]
        ),

        # Motor driver
        Node(
            package='roombot',
            executable='l298n_motor_driver',
            name='l298n_motor_driver',
            output='screen',
            parameters=[driver_params]
        ),
    ])
