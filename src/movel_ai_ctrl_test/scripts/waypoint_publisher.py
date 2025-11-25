#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import time
import math
import sys
import transforms3d as t3d

# Define 5 complex waypoints (x, y, yaw_degrees)
# The sequence is designed to be a complicated route [cite: 7]
WAYPOINTS = [
    (5.0, 0.0, 90.0),    # 1. Forward 5m, then turn 90 deg (facing +Y)
    (5.0, 5.0, 0.0),     # 2. Move to y=5, then turn 0 deg (facing +X)
    (0.0, 5.0, 180.0),   # 3. Sharp turn to x=0, then turn 180 deg (facing -X)
    (0.0, -2.0, 270.0),  # 4. Move to y=-2, then turn 270 deg (facing -Y)
    (-5.0, -5.0, 45.0)   # 5. Long diagonal to Q3, then turn 45 deg
]

def yaw_to_quaternion(yaw_degrees):
    """Converts a Yaw angle (in degrees) to a ROS geometry_msgs/Quaternion using transforms3d."""
    yaw_rad = math.radians(yaw_degrees)
    
    # t3d.euler2quat(roll, pitch, yaw, 'sxyz') returns [w, x, y, z]
    q_t3d = t3d.euler.euler2quat(0, 0, yaw_rad, 'sxyz')
    
    # ROS Quaternion convention is [x, y, z, w]
    return Quaternion(x=q_t3d[1], y=q_t3d[2], z=q_t3d[3], w=q_t3d[0])

def publish_waypoints():
    try:
        rospy.init_node('waypoint_publisher', anonymous=True)
    except rospy.exceptions.InvalidNodeName:
        # Handles case where roscore is not running yet
        pass 
        
    # Standard topic for sending goals to move_base
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    rospy.loginfo("Waiting for ROS Master...")
    time.sleep(3) 

    for i, (x, y, yaw) in enumerate(WAYPOINTS):
        if rospy.is_shutdown():
            break
            
        rospy.loginfo(f"--- Sending Waypoint {i+1}/{len(WAYPOINTS)} ---")
        
        goal = PoseStamped()
        goal.header.frame_id = "map" # Goal frame
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0 
        
        # Convert Yaw to Quaternion
        goal.pose.orientation = yaw_to_quaternion(yaw)
        
        rospy.loginfo(f"Goal: X={x}, Y={y}, Yaw={yaw}°")

        # Publish the goal (multiple times for reliability)
        rospy.Rate(1).sleep()
        for _ in range(5):
            goal_pub.publish(goal)
            rospy.Rate(0.5).sleep()

        # NOTE: Using sleep as a proxy for goal completion [cite: 30]
        rospy.loginfo("Waiting for robot to reach target (Simulated 15s wait)...")
        time.sleep(15) 

    rospy.loginfo("All 5 waypoints have been sent. Task completed.")


if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass