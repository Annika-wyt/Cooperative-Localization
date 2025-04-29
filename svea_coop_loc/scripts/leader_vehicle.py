#! /usr/bin/env python3
import rospy
import numpy as np
import tf2_ros

from message_filters import Subscriber, ApproximateTimeSynchronizer

from riccati_observer import riccati_observer

from std_msgs.msg import Float32, ColorRGBA
from svea_thesis.msg import Aruco, ArucoArray, riccati_setup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, TransformStamped, Point, PointStamped, Quaternion, Vector3, Vector3Stamped, PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu
import tf2_geometry_msgs
import tf.transformations
from svea_thesis.srv import GetPosition, GetPositionResponse

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from pixy_camera.msg import ObjectBearingArrayStamped
from copy import deepcopy

class leader_vehicle():
    def __init__(self):
        rospy.init_node('leader_vehicle')
        self.NAMESPACE = rospy.get_namespace()
        self.SVEANAME = rospy.get_param("~svea_name")
        self.DEBUG = rospy.get_param("~debug", False)

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()
        self.Rate = rospy.Rate(60)
        self.header = None
        # Variables
        self.pose = np.array([None, None, None], dtype=np.float64)

        # Subscriber
        rospy.Subscriber("/qualisys/" + self.SVEANAME + "/pose", PoseStamped, self.poseCallback)
        
        # Publisher
        self.pointPub = rospy.Publisher(f'{self.NAMESPACE}riccatiPoint', PointStamped, queue_size=10)

        # Service
        rospy.Service(f'{self.NAMESPACE}getPositionSrv', GetPosition, self.handleGetPosition)

    def poseCallback(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        self.header = msg.header
        if self.DEBUG:
            print(self.pose)
        
############################################# SERVERS ##############################################
    def handleGetPosition(self, req):
        if req.trigger:
            return GetPositionResponse(self.pose[0],self.pose[1], self.pose[2])
        
############################################# SERVERS ##############################################

    def run(self):
        while not rospy.is_shutdown():
            if not self.header is None:
                pointMsg = PointStamped()
                pointMsg.header = self.header
                pointMsg.point.x = self.pose[0]
                pointMsg.point.y = self.pose[1]
                pointMsg.point.z = self.pose[2]
                self.pointPub.publish(pointMsg)
            self.Rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    leader_vehicle().run()