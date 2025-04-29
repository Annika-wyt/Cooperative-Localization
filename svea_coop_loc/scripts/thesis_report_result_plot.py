#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import do_transform_pose
import csv
import os

# ArUco marker frames
aruco_markers = {
    'aruco38': 'arucoCamera38',
    'aruco39': 'arucoCamera39',
    'aruco40': 'arucoCamera40',
    'aruco41': 'arucoCamera41',
}

camera_frame = 'camera'
base_link_frame = 'base_link'
map_frame = 'map'

csv_file_path = os.path.expanduser("/svea_ws/test_case_2_aruco_markers_38_to_41.csv")

def transform_marker_pose(marker_name, aruco_marker_frame):
    try:
        # Get the transform from base_link to map
        base_to_map_transform = tf_buffer.lookup_transform(map_frame, "base_link_est2", rospy.Time(0), rospy.Duration(0.5))
        
        # Get the transform from aruco_marker to camera
        marker_to_camera_transform = tf_buffer.lookup_transform("camera_est", aruco_marker_frame, rospy.Time(0), rospy.Duration(0.5))
        timestamp = marker_to_camera_transform.header.stamp.to_sec()

        actual_marker_map_transform = tf_buffer.lookup_transform("map", marker_name, rospy.Time(0), rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Failed to find transform for {marker_name}: {e}")
        return

    # Create a dummy pose of the marker in its own frame
    marker_pose = geometry_msgs.msg.PoseStamped()
    marker_pose.header.frame_id = aruco_marker_frame
    marker_pose.pose.position.x = 0.0
    marker_pose.pose.position.y = 0.0
    marker_pose.pose.position.z = 0.0
    marker_pose.pose.orientation.x = 0.0
    marker_pose.pose.orientation.y = 0.0
    marker_pose.pose.orientation.z = 0.0
    marker_pose.pose.orientation.w = 1.0

    # Transform the pose step-by-step
    marker_pose_in_camera = do_transform_pose(marker_pose, marker_to_camera_transform)
    marker_pose_in_base = do_transform_pose(marker_pose_in_camera, camera_to_base_transform)
    marker_pose_in_map = do_transform_pose(marker_pose_in_base, base_to_map_transform)

    # Log marker pose in the map frame
    position = marker_pose_in_map.pose.position
    orientation = marker_pose_in_map.pose.orientation

    # Write data to CSV
    csv_writer.writerow([timestamp, marker_name, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w,
                         actual_marker_map_transform.transform.translation.x, 
                         actual_marker_map_transform.transform.translation.y, 
                         actual_marker_map_transform.transform.translation.z,
                         actual_marker_map_transform.transform.rotation.x, 
                         actual_marker_map_transform.transform.rotation.y, 
                         actual_marker_map_transform.transform.rotation.z, 
                         actual_marker_map_transform.transform.rotation.w])

if __name__ == "__main__":
    rospy.init_node('aruco_transformer')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    get_camera_base = False

    # Wait until the transform from camera to base_link is available
    while not get_camera_base and not rospy.is_shutdown():
        try:
            camera_to_base_transform = tf_buffer.lookup_transform(base_link_frame, camera_frame, rospy.Time(0), rospy.Duration(1.0))
            get_camera_base = True
        except Exception as e:
            rospy.logwarn("Waiting for camera-to-base transform...")

    with open(csv_file_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        # Write the header
        csv_writer.writerow(["timestamp", "marker_name", "est_position_x", "est_position_y", "est_position_z",
                             "est_orientation_x", "est_orientation_y", "est_orientation_z", "est_orientation_w",
                             "act_position_x", "act_position_y", "act_position_z",
                             "act_orientation_x", "act_orientation_y", "act_orientation_z", "act_orientation_w"])

        # Log poses for each marker
        while not rospy.is_shutdown():
            for marker_name, aruco_marker_frame in aruco_markers.items():
                transform_marker_pose(marker_name, aruco_marker_frame)
