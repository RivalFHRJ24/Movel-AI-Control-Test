#!/usr/bin/env python

import rosbag
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import sys, os

# --- Douglas-Peucker Function (Path Simplification) ---
def douglas_peucker(points, epsilon):
    """
    Implements the Douglas-Peucker algorithm for reducing the number of points
    in a curve by removing redundant points while maintaining geometric shape.
    """
    # Find the point with the maximum distance from the line segment
    dmax = 0.0
    index = 0
    end = len(points) - 1
    
    # Helper function to calculate point-to-line distance (2D)
    def point_line_distance(point, start, end):
        if np.all(start == end):
            return np.linalg.norm(point - start)
        return np.abs(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

    for i in range(1, end):
        d = point_line_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d

    # If max distance > epsilon, recursively simplify
    if dmax > epsilon:
        results_left = douglas_peucker(points[:index+1], epsilon)
        results_right = douglas_peucker(points[index:], epsilon)
        
        # Merge results, avoiding duplication of the mid-point
        return np.vstack((results_left[:-1], results_right))
    else:
        # If max distance <= epsilon, keep only start and end points
        return np.vstack((points[0], points[end]))

# --- Moving Average Filter (Noise Mitigation) ---
def moving_average_filter(data, window_size):
    """Applies a Moving Average Filter to smooth the data and mitigate noise."""
    if len(data) == 0:
        return np.array([])
    
    # Padding the data to handle boundary conditions
    padding = np.ones((window_size // 2, data.shape[1])) * data[0]
    padded_data = np.vstack([padding, data, padding])
    
    smoothed_data = np.zeros_like(data)
    for i in range(len(data)):
        start = i
        end = i + window_size
        smoothed_data[i] = np.mean(padded_data[start:end], axis=0)
    return smoothed_data

# --- Main Processing Function ---
def process_rosbag(bag_path, epsilon, N_target=50):
    rospy.init_node('path_simplifier', anonymous=True)
    
    try:
        # Open the rosbag file
        bag = rosbag.Bag(bag_path, 'r')
    except rosbag.BagException:
        rospy.logerr("Failed to open rosbag: %s", bag_path)
        return

    raw_points = []
    # Read messages from the specified topic (from path_test.bag)
    for topic, msg, t in bag.read_messages(topics=['/vslam2d_pose']):
        x = msg.pose.position.x
        y = msg.pose.position.y
        raw_points.append([x, y])
    
    bag.close()
    
    raw_points = np.array(raw_points)
    
    if len(raw_points) < 2:
        rospy.logwarn("Rosbag does not contain enough points.")
        return

    print(f"Raw path loaded: {len(raw_points)} points.")

    # 1. Noise Mitigation (Moving Average Filter)
    window_size = 7 # Tuned parameter for smoothing
    filtered_points = moving_average_filter(raw_points, window_size)
    print(f"Noise mitigated using Moving Average (Window: {window_size}).")

    # 2. Geometric Simplification (Douglas-Peucker)
    simplified_points_dp = douglas_peucker(filtered_points, epsilon)
    print(f"DP simplification (Epsilon: {epsilon}) resulted in: {len(simplified_points_dp)} points.")
    
    # 3. Final Downsampling/Resampling to achieve N_target 
    current_N = len(simplified_points_dp)
    if current_N > N_target:
        # Resample evenly to reach N_target
        indices = np.round(np.linspace(0, current_N - 1, N_target)).astype(int)
        final_points = simplified_points_dp[indices]
    else:
        # If result is already smaller than target, use the DP result (which is superior)
        final_points = simplified_points_dp
        if current_N < N_target:
            print(f"Warning: DP point count ({current_N}) is less than target N ({N_target}). Using DP result.")

    print(f"Final simplified path contains: {len(final_points)} points.")

    # --- Output to CSV File ---
    output_filename = f"simplified_path_N_{len(final_points)}.csv"
    np.savetxt(output_filename, final_points, delimiter=",", header="x,y", comments="")
    print(f"Path saved to: {output_filename}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: rosrun movel_ai_ctrl_test path_simplifier.py <path_to_rosbag> <epsilon_DP> [N_target]")
        print("Example: rosrun movel_ai_ctrl_test path_simplifier.py /home/movel/rosbags/path_test.bag 0.1 50")
        sys.exit(1)

    bag_path = sys.argv[1]
    epsilon = float(sys.argv[2])
    N_target = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    
    process_rosbag(bag_path, epsilon, N_target)