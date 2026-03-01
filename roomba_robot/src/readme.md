# RoomBot - ROS2 Robot Platform

A ROS2 Humble-based robot platform featuring RPLidar A1 laser scanner and BNO055 IMU sensor for autonomous navigation and mapping.

## Hardware Components

- **RPLidar A1** - 360° Laser Scanner
- **BNO055** - 9-DOF Inertial Measurement Unit
- **4-Wheel Differential Drive Base**
- **USB Serial Adapters** for sensor communication

## Features

- Real-time laser scanning with 360° coverage
- IMU data for orientation and motion sensing
- Robot state publishing and visualization
- RViz integration for monitoring and debugging
- Gazebo simulation support

## Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS2 Humble** installed
- **Python 3.10+**
- USB ports for sensor connections

## Installation on New Device

### 1. Install ROS2 Humble (if not already installed)

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-rosdep python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone and Setup Workspace

```bash
# Clone the repository
git clone <your-repository-url> ~/ws_roombot
cd ~/ws_roombot

# Install all package dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install additional system dependencies
sudo apt install -y python3-serial

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
echo "source ~/ws_roombot/install/setup.bash" >> ~/.bashrc
```

### 3. Hardware Setup

#### USB Device Configuration
1. Connect RPLidar A1 to USB port (will appear as `/dev/ttyUSB0` or similar)
2. Connect BNO055 to another USB port (will appear as `/dev/ttyUSB1` or similar)

#### Check Device Connections
```bash
# List USB serial devices
ls -la /dev/ttyUSB*

# Check device permissions (should show dialout group access)
ls -la /dev/ttyUSB*

# Add user to dialout group if needed
sudo usermod -a -G dialout $USER
# Log out and log back in for group changes to take effect
```

#### Verify Device Assignment
```bash
# Check which device is which
lsusb
udevadm info /dev/ttyUSB0
udevadm info /dev/ttyUSB1
```

### 4. Configuration

Update the launch file with correct device paths if needed:
```bash
# Edit the launch file to match your USB device assignments
nano src/roombot/launch/combined_robot.launch.py

# Update these lines to match your devices:
# RPLidar: 'serial_port': '/dev/ttyUSB0'  (or your RPLidar device)
# BNO055: 'uart_port': '/dev/ttyUSB1'     (or your BNO055 device)
```

## Usage

### Launch the Complete System

```bash
# Source the workspace
source ~/ws_roombot/install/setup.bash

# Launch robot with all sensors and RViz
ros2 launch roombot combined_robot.launch.py
```

This will start:
- **Robot State Publisher** - Publishes robot model transforms
- **Joint State Publisher** - Provides wheel joint states for visualization
- **RPLidar Driver** - Publishes laser scan data on `/scan` topic
- **BNO055 Driver** - Publishes IMU data on `/imu/data` topic
- **RViz** - 3D visualization and monitoring interface

### Individual Component Testing

#### Test RPLidar Only
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

#### Test BNO055 Only  
```bash
ros2 launch bno055 bno055_launch.py
```

#### View Robot Model Only
```bash
ros2 launch roombot robot_description.launch.py  # If you create this launch file
```

## Troubleshooting

### Common Issues

#### 1. USB Permission Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### 2. Device Not Found
```bash
# Check connected devices
lsusb
ls -la /dev/ttyUSB*

# Update device paths in launch file accordingly
```

#### 3. RViz Transform Errors
- Ensure all sensors are connected and publishing data
- Check that frame IDs match between URDF and sensor configurations
- Verify `/tf` and `/tf_static` topics are publishing transforms

#### 4. Build Errors
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build
```

### Verify System is Working

#### Check Topics
```bash
# List all active topics
ros2 topic list

# Expected topics:
# /scan (laser data)
# /imu/data (IMU data) 
# /joint_states (wheel positions)
# /tf, /tf_static (transforms)
# /robot_description (robot model)
```

#### Check Transforms
```bash
# View transform tree
ros2 run tf2_tools view_frames
# This creates frames.pdf showing the transform tree

# Check specific transform
ros2 run tf2_ros tf2_echo base_link lidar_link
```

#### Monitor Sensor Data
```bash
# View laser scan data
ros2 topic echo /scan --once

# View IMU data  
ros2 topic echo /imu/data --once

# Check data rates
ros2 topic hz /scan
ros2 topic hz /imu/data
```

## RViz Configuration

The included RViz config displays:
- **Robot Model** - 3D representation with wheels, lidar, and IMU
- **Laser Scan** - Real-time laser points showing obstacles
- **TF Frames** - Coordinate frame visualization
- **Grid** - Reference grid for spatial awareness

### Adding Displays in RViz
1. Click "Add" in RViz Displays panel
2. Select display type (LaserScan, RobotModel, etc.)
3. Set topic names:
   - LaserScan: `/scan`
   - IMU: `/imu/data`

## Package Structure

```
src/roombot/
├── package.xml          # Package dependencies and metadata
├── setup.py            # Python package configuration  
├── launch/             # Launch files
│   └── combined_robot.launch.py
├── urdf/              # Robot description files
│   └── roombot.urdf
├── rviz/              # RViz configuration files
│   └── view_robot.rviz
└── roombot/           # Python package source
    └── __init__.py
```

## Development Notes

- **Frame Configuration**: RPLidar publishes with `frame_id: lidar_link` to match URDF
- **Device Ports**: RPLidar on `/dev/ttyUSB0`, BNO055 on `/dev/ttyUSB1` (adjust as needed)
- **Joint States**: Currently using `joint_state_publisher` for visualization; replace with real motor controllers for actual robot control
- **Simulation Ready**: Gazebo plugins configured in URDF for simulation testing

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes and test thoroughly
4. Submit a pull request with detailed description

## License

[Specify your license here]

## Support

For issues and questions:
- Check the troubleshooting section above
- Review ROS2 Humble documentation
- Check sensor driver documentation (rplidar_ros, bno055)

---

**Last Updated:** January 2026
**ROS2 Version:** Humble
**Tested On:** Ubuntu 22.04 LTS