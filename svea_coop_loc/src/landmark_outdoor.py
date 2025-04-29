#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Point
from svea_thesis.msg import Aruco, ArucoArray

RATE = 150
class landmark_outdoor:
    def __init__(self):
        rospy.init_node('landmark_outdoor')

        self.MapArucoPub = rospy.Publisher('/aruco/detection/Groundtruth', ArucoArray, queue_size=1)    
        self.map_frame = rospy.get_param("~map_frame", "map") #
        self.in_kip = rospy.get_param("~in_kip", True) #

        if self.in_kip:
            self.landmark = {
                10 : [-1.85, 0.85, 0.124],
                11 : [-1.25, 1.00, 0.124],
                12 : [-1.25, 1.50, 0.124],
                13 : [-0.85, 1.85, 0.124],

                14 : [ 0.85, 1.85, 0.124],
                15 : [ 1.30, 1.00, 0.124],
                16 : [ 1.29, 1.50, 0.124],
                17 : [ 1.89, 0.85, 0.124],
                
                18 : [ 1.85, -0.85, 0.124],
                19 : [ 1.25, -1.50, 0.124],
                20 : [ 1.25, -1.00, 0.124],
                21 : [ 0.85, -1.85, 0.124],
                
                22 : [-0.85, -1.85 , 0.124],
                23 : [-1.25, -1.50, 0.124],
                24 : [-1.25 ,-1.00, 0.124],
                25 : [-1.85 ,-0.85, 0.124],
                
                26 : [-3.50 ,-0.50, 0.124],
                27 : [-4.00 , 0.00, 0.124],
                28 : [-3.50 , 0.50, 0.124],
                29 : [-4.00 , 1.00, 0.124],
                30 : [-0.50 ,3.50, 0.124],
                31 : [ 0.00 ,4.00, 0.124],
                32 : [ 0.50 ,3.50, 0.124],
                33 : [ 1.00 ,4.00, 0.124],
                34 : [3.50 , 0.50, 0.124],
                35 : [4.00 , 0.00, 0.124],
                36 : [3.50 , -0.50, 0.124],
                37 : [4.00 , -1.00, 0.124],
                38 : [ 0.50, -3.50, 0.124],
                39 : [-0.00, -4.00, 0.124],
                40 : [-0.50, -3.50, 0.124],
                41 : [-1.00, -4.00, 0.124],
            }
        else:
            self.landmark = {
                15 : [-2.21, -0.412, 0.124],
                16 : [-2.46, -0.144, 0.124],
                17 : [-2.21, -0.135, 0.124],
                18 : [-2.43, -0.441, 0.124],

                19 : [-0.74,  -1.738, 0.124],
                20 : [-1.16,  -1.378, 0.124],
                21 : [-1.16, -0.958, 0.124],
                22 : [-1.71, -0.628, 0.124],

                31 : [0.34, -2.68, 0.124],
                32 : [0.07, -2.907, 0.124],
                33 : [-0.27, -2.612, 0.124],
                34 : [-0.58, -2.915, 0.124],
            }
            # self.landmark = {
# 
                # 15 : [-2.20, -0.60, 0.124],
                # 16 : [-2.90, -0.60, 0.124],
                # 17 : [-3.60, -0.60, 0.124],
                # 18 : [-2.00,  0.60, 0.124],
                # 19 : [-2.70,  0.60, 0.124],
                # 20 : [-3.40,  0.60, 0.124],
                # 21 : [-4.40,  0.60, 0.124],
                # 22 : [-3.40, -2.40, 0.124],
# 
                # 23 : [-4.40, -2.10, 0.124],
                # 24 : [-3.40, -2.90, 0.124],
                # 25 : [-4.40, -2.60, 0.124],
                # 31 : [-4.40 , -3.10, 0.124],
                # 32 : [-3.40 , -3.60, 0.124],
                # 33 : [-4.40 , -3.20, 0.124],
                # 34 : [-3.40 , -4.10, 0.124],
                # 35 : [-4.40 , -3.90, 0.124],
            # }
# 
            # self.landmark = {
            #     10 : [0.727 , 0.320, 0.124],
            #     11 : [1.40 ,  0.320, 0.124],
            #     12 : [2.10 ,  0.320, 0.124],
            #     13 : [0.89 ,  -0.320,0.124],
            #     14 : [1.695 , -0.320, 0.124]
            # }
                # 10 : [-1.04 ,-0.579, 0.124],
                # 11 : [-1.53 , 0.579, 0.124],
                # 12 : [-1.95 ,-0.579, 0.124],
                # 13 : [-2.455 , 0.579,0.124],
                # 14 : [-2.96 ,-0.579, 0.124],
                # 15 : [-3.44 , 0.579, 0.124]

        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.seq = 0

        self.rate = rospy.Rate(RATE)
    
    def FillCovaraince(self):
        TransCov = np.eye(3,6, dtype=float)*1e-3
        RotCov = np.eye(3,6, k=3, dtype=float)*1e-3*5
        return np.vstack((TransCov, RotCov)).flatten()
    
    def addLandmark(self):
        arucoarray = ArucoArray()
        arucoarray.header.stamp = rospy.Time.now()
        arucoarray.header.seq = self.seq
        arucoarray.header.frame_id = self.map_frame
        msg = TransformStamped()
        for key, items in self.landmark.items():
            msg.header = arucoarray.header
            msg.child_frame_id = "aruco" + str(key)
            msg.transform.translation = Vector3(*items)
            msg.transform.rotation = Quaternion(*[0, 0, 0, 1])
            self.static_br.sendTransform(msg)
            rospy.sleep(0.01)

            arucoitem = Aruco()
            arucoitem.marker.header = msg.header
            arucoitem.marker.id = key
            arucoitem.marker.pose.pose.position = Point(*items)
            arucoitem.marker.pose.pose.orientation = Quaternion(*[0, 0, 0, 1])
            arucoitem.marker.pose.covariance = self.FillCovaraince()
            arucoarray.arucos.append(arucoitem)
        return arucoarray
    def run(self):
        while not rospy.is_shutdown():
            ArucoListMSg = self.addLandmark()
            for i in range(RATE):
                self.MapArucoPub.publish(ArucoListMSg)
                self.seq += 1
                ArucoListMSg.header.stamp = rospy.Time.now()
                ArucoListMSg.header.seq = self.seq
                ArucoListMSg.header.frame_id = self.map_frame
                self.rate.sleep()

if __name__ == '__main__':
    landmark_outdoor().run()