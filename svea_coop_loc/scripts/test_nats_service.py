#! /usr/bin/env python3
import rospy
import yaml

import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

from riccati_observer_occlusion import riccati_observer_occlusion

from svea_thesis.msg import riccati_setup
from pixy_camera.msg import ObjectBearing, ObjectBearingArrayStamped
from svea_thesis.srv import GetPosition, GetPositionResponse

from std_msgs.msg import Float32, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, Point, Quaternion, Vector3, PoseStamped, Vector3Stamped, TwistStamped, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu

import tf2_ros
import tf2_geometry_msgs
import tf.transformations

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from copy import deepcopy

class test_nats():
    def __init__(self):
        rospy.init_node('test_nats')
        self.DEBUG = rospy.get_param('~debug', False)

        self.NAMESPACE = rospy.get_namespace()
        self.SVEANAME = rospy.get_param('~svea_name')
        rospy.Service(f'{self.NAMESPACE}getPositionSrv', GetPosition, self.handleGetPosition)
        rospy.Subscriber("/qualisys/" + self.SVEANAME + "/pose", PoseStamped, self.poseCallback)
        
        for idx in range(1, 8):
            try:
                rospy.loginfo(f"waiting for service agent{idx}/getPositionSrv")
                rospy.wait_for_service(f'/agent{idx}/getPositionSrv', timeout=1)
            except:
                rospy.loginfo(f"No service for agent{idx}/getPositionSrv")

    def poseCallback(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        if self.DEBUG:
            print(self.pose)

    def handleGetPosition(self, req):
        if req.trigger:
            return GetPositionResponse(self.pose[0],self.pose[1], self.pose[2])

    def run(self):
        while not rospy.is_shutdown():
            try:
                getPositionSrv = rospy.ServiceProxy('/agent4/getPositionSrv', GetPosition)
                response = getPositionSrv(True)
                print(response)
            except Exception as e:
                print(f"Error : {e}")
        rospy.spin()


if __name__ == '__main__':
    test_nats().run()