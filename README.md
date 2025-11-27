# Distributed Multi-Robot SLAM Stack

This repository contains the **ROS 1 Melodic workspace** for a distributed SLAM dataset project. We collect synchronized multi-robot datasets from ~50 Raspberry Pi-based robots equipped with LiDAR, RGB-D, IMU, and wheel odometry sensors.

## Project Goals

- **Collect synchronized multi-robot datasets** in diverse scenarios (loops, squares, corridors, etc.)
- **Provide a reproducible simulation stack** (Gazebo + RViz) for algorithm development
- **Enable benchmarking** of different distributed SLAM algorithms on the same standardized data

---

## System Specifications

| Component | Details |
|-----------|---------|
| **ROS Version** | ROS 1 Melodic |
| **Target OS** | Ubuntu 18.04 |
| **Hardware** | Raspberry Pi + ROS-compatible sensors |
| **Sensors per Robot** | LiDAR, RGB-D camera, IMU, wheel odometry |
| **Fleet Size** | ~50 robots |

---

## Workspace Organization

All code is organized as a catkin workspace under `catkin_ws/src/`:

```
catkin_ws/src/
├── robot_description/      # URDF/Xacro definitions & sensor frames
├── robot_bringup/          # Single-robot launch files & driver configs
├── robot_sim/              # Gazebo worlds & multi-robot sim launch files
├── robot_viz/              # RViz visualizations for monitoring
├── robot_dataset_tools/    # Dataset processing & validation utilities
├── robot_experiment/       # Experiment scenarios & metadata publishing
└── robot_firmware/         # Robot provisioning & flashing tools (optional)
```

---

## Package Descriptions

### `robot_description`
Contains the robot's kinematic model and sensor frame definitions.

- **URDF/Xacro files** defining the robot structure and sensor attachments
- **Sensor frames**: `base_link`, `lidar_link`, `camera_link`, `imu_link`, etc.
- **Calibration & config files** for LiDAR, RGB-D, and IMU sensor parameters

### `robot_bringup`
Handles the bring-up of a single robot—launching drivers and configuring sensor streaming.

- **Per-robot launch files** that start drivers, TF broadcasting, and sensor nodes
- **Recording launch files** (e.g., `robot_record.launch`) for bag file collection
- **Sensor parameter YAMLs** specifying frame rates, resolutions, and other settings

### `robot_sim`
Simulation environment for development and algorithm testing.

- **Gazebo worlds** for standard scenarios: loops, squares, corridors, etc.
- **Robot models with Gazebo plugins** that mimic real sensor behavior
- **Multi-robot spawn & sim launch files** for testing distributed algorithms

### `robot_viz`
Visualization and monitoring for operators and researchers.

- **RViz configuration files** for single-robot and multi-robot visualization
- **Convenience launch files** to start RViz with pre-configured settings
- **Dashboard-style views** for monitoring fleet state during data collection

### `robot_dataset_tools`
Post-processing tools to turn raw rosbag data into a structured dataset.

- **Validation scripts** for rosbag integrity checks
- **Data extraction & alignment utilities** (timestamp synchronization, etc.)
- **Dataset metadata generation** for reproducible research
- **Custom ROS messages** (e.g., `ExperimentMetadata.msg`)

### `robot_experiment`
Configuration and orchestration of experiments across the fleet.

- **Scenario configurations** (loop, square, corridor, etc.)
- **Experiment metadata node** that broadcasts scenario ID, run ID, robot ID, and conditions
- **Multi-robot launch files** for coordinated experiment execution

### `robot_firmware`
(Optional) Helper tools for fleet provisioning.

- **SD card flashing utilities**
- **Config deployment scripts** for batch updates across robots

---

## Quick Start

**For detailed setup instructions, see [setup.md](setup.md).**

### On Each Robot
```bash
# Clone the workspace
git clone https://github.com/AP-03/Distributed_SLAM.git
cd Distributed_SLAM/catkin_ws

# Build
catkin_make

# Source the setup
source devel/setup.bash
```

### Verify Installation
```bash
rospack find robot_bringup
rospack find robot_sim
rospack find robot_dataset_tools
```

---

## Usage Examples

### Bring Up a Single Robot
```bash
roslaunch robot_bringup robot_bringup.launch robot_id:=robot_01
```

### Start the Simulator
```bash
roslaunch robot_sim multi_robot_sim.launch num_robots:=5
```

### Record a Dataset
```bash
roslaunch robot_bringup robot_record.launch robot_id:=robot_01
```

### Process a Rosbag
```bash
python scripts/process_bagfile.py path/to/bagfile.bag
```

---

## Documentation

- **Setup & Installation**: See [setup.md](setup.md)
- **Robot Calibration**: See `robot_description/config/`
- **Experiment Parameters**: See `robot_experiment/config/experiment_params.yaml`

---

## Contributors

This stack is maintained by the [Multi-Robot SLAM](https://github.com/AP-03) project at UCL (2025–2026).

For questions or issues, please open an issue on GitHub or contact the project maintainers.
