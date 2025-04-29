#! /usr/bin/env python3

import rospy
import tf2_ros
import tf.transformations
import tf2_geometry_msgs

from svea_thesis.msg import Aruco, ArucoArray
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

SUFFIX = "2"

class aruco_change_frame_for_rosbag():
    def __init__(self):
        rospy.init_node("aruco_change_frame_for_rosbag")

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()
        self.visualarr = VMA()

        rospy.Subscriber("/aruco/used", ArucoArray, self.arucoUsedCallback)
        self.ArucoESTPub = rospy.Publisher("/aruco/estimation" + SUFFIX, ArucoArray, queue_size=10)
        
    def arucoUsedCallback(self, msg):
        newmsg = msg
        newmsg.header.frame_id = "camera_est" + SUFFIX
        outputArucoArray = ArucoArray()
        for aruco in newmsg.arucos:
            aruco.marker.header.frame_id = "camera_est" + SUFFIX
            arucotf = TransformStamped()
            arucotf.header = aruco.marker.header
            arucotf.child_frame_id = "arucoFromEST" + str(aruco.marker.id)
            arucotf.transform.translation = aruco.marker.pose.pose.position
            arucotf.transform.rotation = aruco.marker.pose.pose.orientation
            
            outputAruco = Aruco()
            outputAruco.marker.header = aruco.marker.header 
            outputAruco.marker.header.frame_id = "map" 
            outputAruco.marker.id = aruco.marker.id
            
            try:
                transBaseMap = self.buffer.lookup_transform("map", "camera_est" + SUFFIX, arucotf.header.stamp, rospy.Duration(1)) #frame id = svea5, child = camera 
                arucoPose = PoseStamped()
                arucoPose.header = aruco.marker.header
                arucoPose.pose.position = aruco.marker.pose.pose.position
                arucoPose.pose.orientation = aruco.marker.pose.pose.orientation
            except Exception as e:
                print(f"/aruco_change_frame_for_rosbag/arucoUsedCallback: {e}")

            transformed_direction = tf2_geometry_msgs.do_transform_pose(arucoPose, transBaseMap) 
            arucotf = TransformStamped()
            arucotf.header = transformed_direction.header
            arucotf.child_frame_id = "arucoFromEST" + str(aruco.marker.id)
            arucotf.transform.translation = transformed_direction.pose.position
            arucotf.transform.rotation = transformed_direction.pose.orientation
            # self.br.sendTransform(arucotf)
            outputAruco.marker.pose.pose.position = transformed_direction.pose.position
            outputAruco.marker.pose.pose.orientation = transformed_direction.pose.orientation

            outputArucoArray.arucos.append(outputAruco)
        
        self.ArucoESTPub.publish(outputArucoArray)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    aruco_change_frame_for_rosbag().run()