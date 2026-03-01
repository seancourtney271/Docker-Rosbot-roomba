from setuptools import find_packages, setup

package_name = 'roombot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_description.launch.py', 
            'launch/combined_robot.launch.py',
            'launch/test_combined_sensors.launch.py',
            'launch/nav2_bringup.launch.py'
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/roombot.urdf',
            'urdf/roombot_physical.urdf',
            ]),
        ('share/' + package_name + '/rviz', [
            'rviz/view_robot.rviz',
            'rviz/nav2_view.rviz',
            'rviz/simple_nav.rviz'
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/empty.world',
            'worlds/simple_house.world',
            'worlds/maze.world'
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/amcl_params.yaml',
            'config/mapper_params_online_async.yaml',
            'config/ekf.yaml',
            'config/slam_toolbox.yaml',
        ]),
        ('share/' + package_name + '/maps', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seancour',
    maintainer_email='seancour@buffalo.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = roombot.teleop_keyboard:main',
            'encoder_odom_node = roombot.encoder_odom_node:main',
        ],
    },
)
