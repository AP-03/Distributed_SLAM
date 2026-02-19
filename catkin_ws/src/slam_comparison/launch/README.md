# slam_comparison

Compare ground truth (PhaseSpace mocap) with robot wheel odometry trajectories using ROS Melodic.

## Installation

Copy this package into your catkin workspace and build:

```bash
cp -r slam_comparison ~/Distributed_SLAM/catkin_ws/src/
cd ~/Distributed_SLAM/catkin_ws
catkin_make
source devel/setup.bash
```

## Option 1: Live Playback in RViz (Interactive)

Plays both bags simultaneously with a real-time comparison node that shows:
- **Green path + arrow**: Ground truth (mocap)
- **Red path + arrow**: Wheel odometry
- **Yellow line**: Real-time error connecting GT and odom positions
- **White text**: Numerical error in meters
- **Robot URDF** model (optional)
- **Laser scan** visualization

```bash
roslaunch slam_comparison compare_live.launch \
    gt_bag:=$HOME/Distributed_SLAM/catkin_ws/2026-02-18-17-05-12.bag \
    robot_bag:=$HOME/Distributed_SLAM/catkin_ws/agv1_gt_test_20260219_004526.bag
```

### Options
```bash
# Without robot URDF
roslaunch slam_comparison compare_live.launch \
    gt_bag:=/path/to/mocap.bag \
    robot_bag:=/path/to/robot.bag \
    show_robot:=false

# Half speed playback
roslaunch slam_comparison compare_live.launch \
    gt_bag:=/path/to/mocap.bag \
    robot_bag:=/path/to/robot.bag \
    rate:=0.5

# If mocap uses Y-up convention (swap to ROS Z-up)
roslaunch slam_comparison compare_live.launch \
    gt_bag:=/path/to/mocap.bag \
    robot_bag:=/path/to/robot.bag \
    swap_yz:=true
```

### What the node does
1. Subscribes to `/phasespace/rigids` and filters for rigid body ID 1 (`robot_1`)
2. Subscribes to `/odom` for wheel odometry
3. Automatically aligns both trajectories using the first overlapping pose
4. Publishes a `map -> odom` TF so both paths render in the same frame
5. Shows real-time position error

## Option 2: Offline Preprocessing (Plots + Statistics)

Generates publication-quality plots and error metrics without needing live playback:

```bash
roslaunch slam_comparison preprocess.launch \
    gt_bag:=$HOME/Distributed_SLAM/catkin_ws/2026-02-18-17-05-12.bag \
    robot_bag:=$HOME/Distributed_SLAM/catkin_ws/agv1_gt_test_20260219_004526.bag \
    output_dir:=$HOME/Distributed_SLAM/comparison_output
```

Or run the script directly:
```bash
python scripts/preprocess_bags.py \
    --gt_bag /path/to/mocap.bag \
    --robot_bag /path/to/robot.bag \
    --output_dir ./comparison_output
```

### Outputs
| File | Description |
|------|-------------|
| `trajectory_comparison.png` | Top-down 2D plot: GT (green) vs Odom (red) with error lines |
| `error_over_time.png` | 2D and 3D position error plotted over time |
| `error_heatmap.png` | GT trajectory colored by error magnitude |
| `aligned_poses.csv` | Time-aligned CSV with GT, odom, and error columns |
| `statistics.txt` | ATE RMSE, mean, max, min, std for 2D and 3D |

## Troubleshooting

### Trajectories don't overlap at all
- Your mocap might output in **millimeters**. Uncomment the mm->m conversion lines in both scripts:
  ```python
  # x, y, z = x / 1000.0, y / 1000.0, z / 1000.0
  ```
- Your mocap might use **Y-up** convention. Try `swap_yz:=true`

### No GT poses found
- Verify the rigid body ID: `rostopic echo /phasespace/rigids` and check which IDs appear
- Default is ID=1, change with `rigid_id:=<your_id>`

### Alignment looks wrong
- The auto-alignment uses the first overlapping poses. If the robot was moving at bag start, the alignment may be slightly off. Try starting bags a bit before the robot moves.

### TF errors in RViz
- The node publishes `map -> odom`. If the bag already publishes `odom -> base_link`, the full chain `map -> odom -> base_link` should work.
- If you see "no transform" errors, check: `rosrun tf tf_monitor`

## Package Structure
```
slam_comparison/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── comparison.rviz      # Pre-configured RViz layout
├── launch/
│   ├── compare_live.launch   # Live playback + RViz
│   └── preprocess.launch     # Offline analysis
└── scripts/
    ├── gt_comparison_node.py # Real-time comparison ROS node
    └── preprocess_bags.py    # Offline trajectory analysis
```
