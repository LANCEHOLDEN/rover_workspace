# Rover Workspace

ROS 2 Humble workspace for an outdoor autonomous rover using a Rover Robotics Zero 2, InertialSense IMX-5 (GPS/INS), and RPLiDAR.

---

## Hardware

| Component | Details |
|---|---|
| Rover | Rover Robotics Zero 2 |
| GPS/INS | InertialSense IMX-5 |
| LiDAR | RPLiDAR |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |

---

## Package Overview

| Package | Purpose |
|---|---|
| `roverrobotics_ros2` | Rover drive stack — motor control, wheel odometry, URDF |
| `inertial_sense_ros2` | GPS/INS driver — publishes `/ins`, `/imu`, `/gps` |
| `waypoint_reader` | Waypoint follower node |
| `rplidar_ros` | RPLiDAR driver |
| `bno055` | BNO055 IMU driver |
| `move_forward` / `move_5_feet` | Simple motion test nodes |

---

## Key Topics

| Topic | Message Type | Source |
|---|---|---|
| `/ins` | `nav_msgs/Odometry` | InertialSense (position + orientation + velocity) |
| `/imu` | `sensor_msgs/Imu` | InertialSense (raw IMU) |
| `/gps` | `inertial_sense_ros2/msg/GPS` | InertialSense (lat/lon/alt) |
| `/odometry/wheels` | `nav_msgs/Odometry` | Rover driver (wheel encoder odom) |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF fusion output |
| `/cmd_vel` | `geometry_msgs/Twist` | Waypoint follower → rover driver |

---

## Setup

### 1. Prerequisites

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-robot-localization ros-humble-navigation2 python3-colcon-common-extensions
```

### 2. Clone the workspace

```bash
git clone https://github.com/YOUR_USERNAME/rover_workspace.git
cd rover_workspace
```

### 3. Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
colcon build
source install/setup.bash
```

Add to `~/.bashrc` for convenience:
```bash
echo "source ~/rover_workspace/install/setup.bash" >> ~/.bashrc
```

---

## Launch Files

### INS + Wheel Odometry Fusion (recommended)

Starts the rover driver, InertialSense node, and EKF all at once.
Publishes a fused `/odometry/filtered` topic.

```bash
ros2 launch roverrobotics_driver ins_fusion.launch.py
```

Optional arguments:
```bash
ros2 launch roverrobotics_driver ins_fusion.launch.py port:=/dev/ttyUSB0 baudrate:=921600
```

### Rover Driver Only

```bash
ros2 launch roverrobotics_driver zero.launch.py
```

### InertialSense Only

```bash
ros2 launch inertial_sense_ros2 inertial_sense.launch.py port:=/dev/ttyAMC2
```

### Waypoint Follower

```bash
ros2 launch waypoint_reader waypoint_follower.launch.py
```

---

## Utility Scripts (workspace root)

### Log Waypoints

Drive the rover to a position, press **SPACE** to save it, **q** to write the file.

```bash
python3 log_waypoints.py
python3 log_waypoints.py -o my_waypoints.yaml
```

### GPS to RViz

Converts GPS lat/lon to local ENU frame for visualization in RViz2.
Publishes `/gps_path`, `/gps_pose`, and `/rtk_markers`.

```bash
python3 gps_to_rviz.py
```

### Extract Waypoints from Bag

```bash
python3 extract_waypoints.py <bag_file> -o waypoints.yaml
```

### NTRIP RTK Proxy

Multiplexes GEODNET corrections into a local TCP server for the InertialSense node.

```bash
python3 ntrip_proxy.py
```

Then set in your launch file:
```
RTK_server_IP: '127.0.0.1'
RTK_server_port: 2102
```

---

## Waypoint File Format

```yaml
frame_id: odom
waypoints:
  - x: 3868635.25
    y: -9148046.0
    z: -258.76
    yaw: 0.5167
```

---

## EKF Sensor Fusion

The `ins_fusion.launch.py` launch file uses `robot_localization` to fuse:

| Sensor | Topic | Variables used |
|---|---|---|
| Wheel odometry | `/odometry/wheels` | Forward velocity (vx) |
| INS | `/ins` | Yaw heading, forward velocity, yaw rate |
| IMU | `/imu` | Yaw rate |

Config file: `src/roverrobotics_ros2/roverrobotics_driver/config/localization_ins_fusion.yaml`

Output: `/odometry/filtered` — use this as your odometry source for navigation.

---

## Device Ports

| Device | Default Port |
|---|---|
| Rover drive | `/dev/rover-control` |
| InertialSense | `/dev/ttyAMC2` |

To find your port:
```bash
ls /dev/tty*
```
