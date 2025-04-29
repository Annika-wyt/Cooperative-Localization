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

class pose_service():
    def __init__(self):
        rospy.init_node('pose_service')
        self.NAMESPACE = rospy.get_namespace()
        self.LEADERNAME = rospy.get_param("~leader_name")
        self.LEADERAGENT = rospy.get_param("~leader_agent")
        self.DEBUG = rospy.get_param("~debug", False)

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        # Variables
        self.pose = np.array([None, None, None], dtype=np.float64)

        # Subscriber
        rospy.Subscriber("/qualisys/" + self.LEADERNAME + "/pose", PoseStamped, self.poseCallback)
        
        # Service
        rospy.Service(f'{self.LEADERAGENT}/getPositionSrv', GetPosition, self.handleGetPosition)

    def poseCallback(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        if self.DEBUG:
            print(self.pose)
        
############################################# SERVERS ##############################################
    def handleGetPosition(self, req):
        if req.trigger:
            a = GetPositionResponse(self.pose[0],self.pose[1], self.pose[2])
            print(a)
            return a #GetPositionResponse(self.pose[0],self.pose[1], self.pose[2])
        
############################################# SERVERS ##############################################

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    pose_service().run()