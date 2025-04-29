#! /usr/bin/env python3
import psutil
import os

import rospy
import numpy as np
import tf2_ros

from message_filters import Subscriber, ApproximateTimeSynchronizer

from riccati_observer import riccati_observer

from std_msgs.msg import Float32, ColorRGBA
from svea_thesis.msg import Aruco, ArucoArray, riccati_setup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, Point, Quaternion, Vector3, PoseStamped, Vector3Stamped, TwistStamped, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu
import tf2_geometry_msgs
import tf.transformations

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from pixy_camera.msg import ObjectBearingArrayStamped
from copy import deepcopy

class pixy_camera_test():
    def __init__(self):
        rospy.init_node('pixy_camera_test')

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber("/agent8/pixy_landmark_detection/objectsBearing", ObjectBearingArrayStamped, self.visualDirection)


    def visualDirection(self, msg):
        AllTransforms = []
        for idx in range(len(msg.bearing)):
            Tmsg = TransformStamped()
            Tmsg.header = msg.header
            Tmsg.header.frame_id = "camera"
            Tmsg.child_frame_id = 'Direction' + str(msg.bearing[idx].id)
            Tmsg.transform.translation = Vector3(*[msg.bearing[idx].bearing_x, msg.bearing[idx].bearing_y, msg.bearing[idx].bearing_z])
            Tmsg.transform.rotation = Quaternion(*[0, 0, 0, 1]) #x, y, z, w
            AllTransforms.append(deepcopy(Tmsg))
        self.sbr.sendTransform(AllTransforms)

    def run(self):
        while not rospy.is_shutdown():
            Tmsg = TransformStamped()
            Tmsg.header.stamp = rospy.Time.now()
            Tmsg.header.frame_id = "map"
            Tmsg.child_frame_id = "camera"
            Tmsg.transform.translation = Vector3(*[0, 0, 0])
            qua = np.array([-1.0, 1.0, -1.0, 1.0], dtype=np.float64)
            qua /= np.linalg.norm(qua)
            Tmsg.transform.rotation = Quaternion(*qua) #x, y, z, w
            self.br.sendTransform(Tmsg)
        rospy.spin()

if __name__ == '__main__':
    pixy_camera_test().run()