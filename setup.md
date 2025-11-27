# Installation & Setup Guide

**Project:** Multi-Robot SLAM Dataset (UCL 2025–2026)  
**Platform:** myAGV (Raspberry Pi + ROS Melodic)  
**Target OS:** Ubuntu 18.04

This guide enables all robots to reliably collect SLAM-quality datasets for multi-robot experiments. Follow these instructions **on each robot**.

---

## Prerequisites

Before starting, ensure you have:

- A Raspberry Pi running **Ubuntu 18.04**
- **Sudo access** on the robot
- **Git** installed (if not, run: `sudo apt install -y git`)
- A working **internet connection**

---

## Step 1: Update System Packages

```bash
sudo apt update
sudo apt upgrade -y
```

---

## Step 2: Install ROS Melodic

Add the ROS repository:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
```

Install the ROS signing key:

```bash
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Update and install ROS:

```bash
sudo apt update
sudo apt install -y ros-melodic-ros-base
```

Initialize rosdep (required for dependency management):

```bash
sudo rosdep init
rosdep update
```

Source ROS in your shell:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3: Install Required ROS Packages

Install essential ROS packages for the SLAM stack:

```bash
sudo apt install -y \
  ros-melodic-tf \
  ros-melodic-tf2-ros \
  ros-melodic-serial \
  ros-melodic-robot-state-publisher \
  ros-melodic-urg-node \
  ros-melodic-rosbag \
  python-catkin-tools \
  build-essential
```

---

## Step 4: Verify Vendor Workspace

The robot's manufacturer provides the myAGV driver packages in a separate workspace:

```bash
ls ~/myagv_ros/src
```

You should see the myAGV driver packages (odometry, base controller, etc.) here. **Do not modify this workspace**—our code lives in a separate catkin workspace.

---

## Step 5: Clone the Distributed SLAM Workspace

Clone the repository into your home directory:

```bash
cd ~
git clone https://github.com/AP-03/Distributed_SLAM.git
cd Distributed_SLAM/catkin_ws
```

---

## Step 6: Build the Workspace

Build all packages using catkin:

```bash
catkin_make
```

This will compile all packages in `catkin_ws/src/` and generate the setup scripts in `devel/`.

**Build Tips:**
- If you encounter missing dependencies, run `rosdep install --from-paths src --ignore-src -r -y`
- Builds may take 5–10 minutes on a Raspberry Pi

---

## Step 7: Source the Workspace

After building, source the workspace setup script:

```bash
source ~/Distributed_SLAM/catkin_ws/devel/setup.bash
```

To avoid sourcing manually in every new terminal, add it to your shell configuration:

```bash
echo "source ~/Distributed_SLAM/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## Step 8: Verify Installation

Confirm that ROS can find the packages:

```bash
rospack find robot_bringup
rospack find robot_sim
rospack find robot_dataset_tools
```

Each command should print the path to the package. If successful, you'll see output like:

```
/home/pi/Distributed_SLAM/catkin_ws/src/robot_bringup
/home/pi/Distributed_SLAM/catkin_ws/src/robot_sim
/home/pi/Distributed_SLAM/catkin_ws/src/robot_dataset_tools
```

---

## Common Issues & Troubleshooting

### Issue: `command not found: catkin_make`
**Solution:** Install catkin tools:
```bash
sudo apt install -y python-catkin-tools
```

### Issue: `Could not find a package` error
**Solution:** Make sure you've sourced the workspace:
```bash
source ~/Distributed_SLAM/catkin_ws/devel/setup.bash
```

### Issue: Build fails with missing dependencies
**Solution:** Install missing dependencies automatically:
```bash
cd ~/Distributed_SLAM/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Issue: `permission denied` during `apt` commands
**Solution:** Use `sudo` (already included in the commands above)

---

## Next Steps

Once installation is complete:

1. **Configure your robot:** Edit sensor configurations in `robot_description/config/`
2. **Test the bringup:** Run `roslaunch robot_bringup robot_bringup.launch robot_id:=robot_01`
3. **Verify sensors:** Use `rostopic list` to see published sensor topics
4. **Record data:** Use `roslaunch robot_bringup robot_record.launch robot_id:=robot_01` to start collecting datasets

For more details, refer to the main [README.md](README.md) and individual package READMEs.

---

## Questions & Support

If you encounter issues during setup:

1. Check this guide's troubleshooting section
2. Review package-specific documentation in `catkin_ws/src/*/README.md`
3. Open an issue on the [GitHub repository](https://github.com/AP-03/Distributed_SLAM/issues)
4. Contact the project maintainers

---

**Happy exploring! 🚀**
