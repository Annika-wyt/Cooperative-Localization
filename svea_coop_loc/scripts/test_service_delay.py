#! /usr/bin/env python3
import rospy
import numpy as np
import tf2_ros

from message_filters import Subscriber, ApproximateTimeSynchronizer

from riccati_observer import riccati_observer

from std_msgs.msg import Float32, ColorRGBA
from svea_thesis.msg import Aruco, ArucoArray, riccati_setup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, TransformStamped, Point, Quaternion, Vector3, Vector3Stamped, PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu
import tf2_geometry_msgs
import tf.transformations
from svea_thesis.srv import GetPosition, GetPositionResponse

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from pixy_camera.msg import ObjectBearingArrayStamped
from copy import deepcopy

class test_pose_service():
    def __init__(self):
        rospy.init_node('test_pose_service')

        rospy.wait_for_service(f'/agent4/getPositionSrv', timeout=0.05)
        self.pub_pose = rospy.Publisher("/test_pose_service", Point, queue_size=10)
        self.doing = False

    def call_service(self):
        self.doing = True
        temp_timer = rospy.Time.now()
        getPositionSrv = rospy.ServiceProxy(f'/agent4/getPositionSrv', GetPosition)
        response = getPositionSrv(True)
        print((rospy.Time.now()-temp_timer).to_sec())
        msg = Point()
        msg.x = response.pose_x
        msg.y = response.pose_y
        msg.z = response.pose_z
        self.pub_pose.publish(msg)
        self.doing = False

    def run(self):
        while not rospy.is_shutdown():
            if not self.doing:
                self.call_service()

if __name__ == "__main__":
    test_pose_service().run()