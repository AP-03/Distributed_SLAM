#!/usr/bin/env python
"""
preprocess_bags.py
------------------
Offline tool to extract and compare ground truth (mocap) and odometry
trajectories from two rosbag files. Produces:
  1. 2D top-down trajectory plot (GT vs odom)
  2. Position error over time plot
  3. CSV of time-aligned poses and errors
  4. Summary statistics (ATE, max error, etc.)

Usage:
  rosrun slam_comparison preprocess_bags.py \
      _gt_bag:=/path/to/mocap.bag \
      _robot_bag:=/path/to/robot.bag \
      _output_dir:=/path/to/output

Or standalone:
  python preprocess_bags.py --gt_bag /path/to/mocap.bag \
                            --robot_bag /path/to/robot.bag \
                            --output_dir ./output

Works with ROS Melodic (Python 2.7 compatible).
"""

from __future__ import print_function
import os
import sys
import math
import argparse
import numpy as np

try:
    import rospy
    import rosbag
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: rospy/rosbag not found. Run within a ROS environment.")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("WARNING: matplotlib not found. Install with: pip install matplotlib")

try:
    import tf.transformations as tft
except ImportError:
    # Minimal fallback
    tft = None


def quaternion_matrix_simple(q):
    """Simple quaternion to 4x4 matrix if tf.transformations unavailable."""
    if tft is not None:
        return tft.quaternion_matrix(q)
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw),     0],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw),     0],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy), 0],
        [0, 0, 0, 1]
    ])


def quaternion_from_matrix_simple(M):
    if tft is not None:
        return tft.quaternion_from_matrix(M)
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    else:
        if M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
            s = 2.0 * math.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
            w = (M[2, 1] - M[1, 2]) / s
            x = 0.25 * s
            y = (M[0, 1] + M[1, 0]) / s
            z = (M[0, 2] + M[2, 0]) / s
        elif M[1, 1] > M[2, 2]:
            s = 2.0 * math.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
            w = (M[0, 2] - M[2, 0]) / s
            x = (M[0, 1] + M[1, 0]) / s
            y = 0.25 * s
            z = (M[1, 2] + M[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
            w = (M[1, 0] - M[0, 1]) / s
            x = (M[0, 2] + M[2, 0]) / s
            y = (M[1, 2] + M[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


def extract_gt_poses(bag_path, rigid_id=1, topic='/phasespace/rigids', swap_yz=False):
    """Extract ground truth poses from the mocap bag."""
    print("Reading GT bag: {}".format(bag_path))
    poses = []  # list of (time_sec, x, y, z, qx, qy, qz, qw)

    bag = rosbag.Bag(bag_path, 'r')
    count = 0
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        for rigid in msg.rigids:
            if rigid.id == rigid_id:
                x, y, z = rigid.x, rigid.y, rigid.z
                qw, qx, qy, qz = rigid.qw, rigid.qx, rigid.qy, rigid.qz

                if swap_yz:
                    x, y, z = x, -z, y
                    qx, qy, qz, qw = qx, -qz, qy, qw

                # Uncomment if mocap outputs mm:
                # x, y, z = x / 1000.0, y / 1000.0, z / 1000.0

                time_sec = msg.header.stamp.to_sec()
                if time_sec == 0:
                    time_sec = t.to_sec()
                poses.append((time_sec, x, y, z, qx, qy, qz, qw))
                count += 1
    bag.close()
    print("  Extracted {} GT poses for rigid body ID {}".format(count, rigid_id))
    return poses


def extract_odom_poses(bag_path, topic='/odom'):
    """Extract odometry poses from the robot bag."""
    print("Reading robot bag: {}".format(bag_path))
    poses = []

    bag = rosbag.Bag(bag_path, 'r')
    count = 0
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        time_sec = msg.header.stamp.to_sec()
        if time_sec == 0:
            time_sec = t.to_sec()
        poses.append((time_sec, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        count += 1
    bag.close()
    print("  Extracted {} odom poses".format(count))
    return poses


def compute_alignment(gt_poses, odom_poses):
    """
    Compute rigid alignment from the first overlapping poses.
    Returns 4x4 transformation matrix T such that:
        P_gt = T * P_odom
    """
    # Find time overlap
    gt_start = gt_poses[0][0]
    odom_start = odom_poses[0][0]
    common_start = max(gt_start, odom_start)

    # Get first GT and odom poses near the common start
    gt_first = min(gt_poses, key=lambda p: abs(p[0] - common_start))
    odom_first = min(odom_poses, key=lambda p: abs(p[0] - common_start))

    T_gt = quaternion_matrix_simple([gt_first[4], gt_first[5], gt_first[6], gt_first[7]])
    T_gt[0, 3] = gt_first[1]
    T_gt[1, 3] = gt_first[2]
    T_gt[2, 3] = gt_first[3]

    T_odom = quaternion_matrix_simple([odom_first[4], odom_first[5], odom_first[6], odom_first[7]])
    T_odom[0, 3] = odom_first[1]
    T_odom[1, 3] = odom_first[2]
    T_odom[2, 3] = odom_first[3]

    T_align = T_gt.dot(np.linalg.inv(T_odom))
    print("  Alignment computed.")
    print("  GT first pose time: {:.3f}s, Odom first pose time: {:.3f}s".format(
        gt_first[0], odom_first[0]))
    return T_align


def transform_poses(poses, T):
    """Apply 4x4 transformation to all poses."""
    transformed = []
    for pose in poses:
        time_sec = pose[0]
        T_p = quaternion_matrix_simple([pose[4], pose[5], pose[6], pose[7]])
        T_p[0, 3] = pose[1]
        T_p[1, 3] = pose[2]
        T_p[2, 3] = pose[3]

        T_new = T.dot(T_p)
        q = quaternion_from_matrix_simple(T_new)
        transformed.append((
            time_sec,
            T_new[0, 3], T_new[1, 3], T_new[2, 3],
            q[0], q[1], q[2], q[3]
        ))
    return transformed


def time_align(gt_poses, odom_poses, max_dt=0.05):
    """
    For each odom pose, find the nearest GT pose in time.
    Returns list of (time, gt_x, gt_y, gt_z, odom_x, odom_y, odom_z, error_3d, error_2d).
    """
    gt_times = np.array([p[0] for p in gt_poses])
    aligned = []

    for op in odom_poses:
        ot = op[0]
        idx = np.argmin(np.abs(gt_times - ot))
        dt = abs(gt_times[idx] - ot)
        if dt > max_dt:
            continue

        gp = gt_poses[idx]
        dx = gp[1] - op[1]
        dy = gp[2] - op[2]
        dz = gp[3] - op[3]
        err_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        err_2d = math.sqrt(dx*dx + dy*dy)

        aligned.append((ot, gp[1], gp[2], gp[3], op[1], op[2], op[3], err_3d, err_2d))

    return aligned


def compute_statistics(aligned):
    """Compute ATE and other error statistics."""
    errors_3d = [a[7] for a in aligned]
    errors_2d = [a[8] for a in aligned]

    stats = {
        'num_matched': len(aligned),
        'ate_3d_rmse': math.sqrt(sum(e*e for e in errors_3d) / len(errors_3d)),
        'ate_3d_mean': sum(errors_3d) / len(errors_3d),
        'ate_3d_max': max(errors_3d),
        'ate_3d_min': min(errors_3d),
        'ate_3d_std': float(np.std(errors_3d)),
        'ate_2d_rmse': math.sqrt(sum(e*e for e in errors_2d) / len(errors_2d)),
        'ate_2d_mean': sum(errors_2d) / len(errors_2d),
        'ate_2d_max': max(errors_2d),
        'duration_sec': aligned[-1][0] - aligned[0][0],
    }
    return stats


def plot_trajectories(gt_poses, odom_aligned_poses, aligned, output_dir):
    """Generate trajectory and error plots."""
    if not HAS_MPL:
        print("Skipping plots (matplotlib not installed)")
        return

    gt_x = [p[1] for p in gt_poses]
    gt_y = [p[2] for p in gt_poses]
    odom_x = [p[1] for p in odom_aligned_poses]
    odom_y = [p[2] for p in odom_aligned_poses]

    # ---- Plot 1: Top-down trajectory comparison ----
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    ax.plot(gt_x, gt_y, 'g-', linewidth=2, label='Ground Truth (mocap)', alpha=0.9)
    ax.plot(odom_x, odom_y, 'r-', linewidth=1.5, label='Wheel Odometry', alpha=0.8)

    # Mark start and end
    ax.plot(gt_x[0], gt_y[0], 'go', markersize=12, label='GT Start')
    ax.plot(gt_x[-1], gt_y[-1], 'gs', markersize=12, label='GT End')
    ax.plot(odom_x[0], odom_y[0], 'ro', markersize=10, label='Odom Start')
    ax.plot(odom_x[-1], odom_y[-1], 'rs', markersize=10, label='Odom End')

    # Draw error lines every N points
    skip = max(1, len(aligned) // 30)
    for i in range(0, len(aligned), skip):
        a = aligned[i]
        ax.plot([a[1], a[4]], [a[2], a[5]], 'y-', linewidth=0.8, alpha=0.6)

    ax.set_xlabel('X (m)', fontsize=13)
    ax.set_ylabel('Y (m)', fontsize=13)
    ax.set_title('Ground Truth vs Odometry - Top Down View', fontsize=15)
    ax.legend(fontsize=11, loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    traj_path = os.path.join(output_dir, 'trajectory_comparison.png')
    fig.savefig(traj_path, dpi=150)
    plt.close(fig)
    print("  Saved: {}".format(traj_path))

    # ---- Plot 2: Error over time ----
    t0 = aligned[0][0]
    times = [a[0] - t0 for a in aligned]
    err_3d = [a[7] for a in aligned]
    err_2d = [a[8] for a in aligned]

    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax1.plot(times, err_2d, 'b-', linewidth=1.2, label='2D Error (XY)')
    ax1.fill_between(times, 0, err_2d, alpha=0.15, color='blue')
    ax1.set_ylabel('2D Error (m)', fontsize=12)
    ax1.set_title('Position Error Over Time', fontsize=14)
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)

    ax2.plot(times, err_3d, 'r-', linewidth=1.2, label='3D Error (XYZ)')
    ax2.fill_between(times, 0, err_3d, alpha=0.15, color='red')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('3D Error (m)', fontsize=12)
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    err_path = os.path.join(output_dir, 'error_over_time.png')
    fig2.savefig(err_path, dpi=150)
    plt.close(fig2)
    print("  Saved: {}".format(err_path))

    # ---- Plot 3: Error heatmap on trajectory ----
    fig3, ax3 = plt.subplots(1, 1, figsize=(12, 10))
    gt_xs = [a[1] for a in aligned]
    gt_ys = [a[2] for a in aligned]
    errs = [a[8] for a in aligned]  # 2D error for color

    sc = ax3.scatter(gt_xs, gt_ys, c=errs, cmap='RdYlGn_r', s=8, alpha=0.8)
    cbar = plt.colorbar(sc, ax=ax3)
    cbar.set_label('2D Position Error (m)', fontsize=12)
    ax3.set_xlabel('X (m)', fontsize=13)
    ax3.set_ylabel('Y (m)', fontsize=13)
    ax3.set_title('Error Heatmap Along GT Trajectory', fontsize=15)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    plt.tight_layout()
    heat_path = os.path.join(output_dir, 'error_heatmap.png')
    fig3.savefig(heat_path, dpi=150)
    plt.close(fig3)
    print("  Saved: {}".format(heat_path))


def save_csv(aligned, output_dir):
    """Save time-aligned data to CSV."""
    csv_path = os.path.join(output_dir, 'aligned_poses.csv')
    with open(csv_path, 'w') as f:
        f.write("time_sec,gt_x,gt_y,gt_z,odom_x,odom_y,odom_z,error_3d,error_2d\n")
        for a in aligned:
            f.write("{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(*a))
    print("  Saved: {}".format(csv_path))


def save_stats(stats, output_dir):
    """Print and save summary statistics."""
    stats_str = """
=====================================
  TRAJECTORY COMPARISON STATISTICS
=====================================
Matched pose pairs:   {num_matched}
Duration:             {duration_sec:.1f} s

--- Absolute Trajectory Error (ATE) ---
  3D RMSE:   {ate_3d_rmse:.4f} m
  3D Mean:   {ate_3d_mean:.4f} m
  3D Max:    {ate_3d_max:.4f} m
  3D Min:    {ate_3d_min:.4f} m
  3D Std:    {ate_3d_std:.4f} m

  2D RMSE:   {ate_2d_rmse:.4f} m
  2D Mean:   {ate_2d_mean:.4f} m
  2D Max:    {ate_2d_max:.4f} m
=====================================
""".format(**stats)

    print(stats_str)
    stats_path = os.path.join(output_dir, 'statistics.txt')
    with open(stats_path, 'w') as f:
        f.write(stats_str)
    print("  Saved: {}".format(stats_path))


def main():
    # --- Parse arguments (support both ROS params and CLI args) ---
    gt_bag = None
    robot_bag = None
    output_dir = None
    rigid_id = 1
    swap_yz = False

    # Try ROS params first
    if HAS_ROS:
        try:
            rospy.init_node('preprocess_bags', anonymous=True)
            gt_bag = rospy.get_param('~gt_bag', None)
            robot_bag = rospy.get_param('~robot_bag', None)
            output_dir = rospy.get_param('~output_dir', None)
            rigid_id = rospy.get_param('~rigid_body_id', 1)
            swap_yz = rospy.get_param('~swap_mocap_yz', False)
        except Exception:
            pass

    # Fall back to CLI args
    if gt_bag is None:
        parser = argparse.ArgumentParser(description='Preprocess GT and odom bags')
        parser.add_argument('--gt_bag', required=True, help='Path to mocap bag file')
        parser.add_argument('--robot_bag', required=True, help='Path to robot bag file')
        parser.add_argument('--output_dir', default='./comparison_output',
                            help='Output directory for plots and CSV')
        parser.add_argument('--rigid_id', type=int, default=1,
                            help='Rigid body ID to track (default: 1)')
        parser.add_argument('--swap_yz', action='store_true',
                            help='Swap Y/Z axes for mocap data')
        args = parser.parse_args()
        gt_bag = args.gt_bag
        robot_bag = args.robot_bag
        output_dir = args.output_dir
        rigid_id = args.rigid_id
        swap_yz = args.swap_yz

    if not output_dir:
        output_dir = './comparison_output'

    os.makedirs(output_dir, exist_ok=False) if not os.path.exists(output_dir) else None

    print("\n=== Ground Truth vs Odometry Comparison ===")
    print("GT bag:    {}".format(gt_bag))
    print("Robot bag: {}".format(robot_bag))
    print("Output:    {}".format(output_dir))
    print("Rigid ID:  {}".format(rigid_id))
    print("")

    # --- Extract ---
    gt_poses = extract_gt_poses(gt_bag, rigid_id=rigid_id, swap_yz=swap_yz)
    odom_poses = extract_odom_poses(robot_bag)

    if len(gt_poses) == 0:
        print("ERROR: No GT poses found for rigid body ID {}!".format(rigid_id))
        return
    if len(odom_poses) == 0:
        print("ERROR: No odom poses found!")
        return

    # --- Align ---
    print("\nComputing alignment...")
    T_align = compute_alignment(gt_poses, odom_poses)
    odom_aligned = transform_poses(odom_poses, T_align)

    # --- Time-align and compute errors ---
    print("Time-aligning poses...")
    aligned = time_align(gt_poses, odom_aligned, max_dt=0.1)
    if len(aligned) == 0:
        print("ERROR: No time-matched poses found! Check that bags overlap in time.")
        return
    print("  Matched {} pose pairs".format(len(aligned)))

    # --- Statistics ---
    stats = compute_statistics(aligned)
    save_stats(stats, output_dir)

    # --- Plots ---
    print("\nGenerating plots...")
    plot_trajectories(gt_poses, odom_aligned, aligned, output_dir)

    # --- CSV ---
    save_csv(aligned, output_dir)

    print("\nDone! All outputs in: {}".format(output_dir))


if __name__ == '__main__':
    main()
