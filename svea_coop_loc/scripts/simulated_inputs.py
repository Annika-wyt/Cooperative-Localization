#! /usr/bin/env python3
import psutil
import os

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

class riccati_occlusion_simulated_topics():
    def __init__(self):
        rospy.init_node('riccati_estimation_occlusion')
        self.NAMESPACE = rospy.get_namespace()

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        # Publisher
        self.landmarkPub = rospy.Publisher(self.NAMESPACE + "pixy_node/objectsBearing", ObjectBearingArrayStamped, queue_size=10)
        self.twistPub = rospy.Publisher("qualisys/svea3/velocity", TwistStamped, queue_size=10)
        self.yaml_file_path = rospy.get_param('~env_yaml', "")

        self.hdmap = {}

        self.poseFormula = lambda t: np.transpose(np.array([[2.5 + 2.5*np.cos(t), 2.5*np.sin(t), 5]]))
        self.linVelFormula = lambda t: np.transpose(np.array([[-2.5*np.sin(t), 2.5*np.cos(t), 0]]))
        self.angVelFormula = lambda t: np.transpose(np.array([[0.1*np.sin(t), 0.4*np.cos(2*t), 0.6*t]]))

        self.starttime = None
        self.currenttime = None
        self.initRot = np.eye(3)
        self.load_yaml()

    def load_yaml(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
            
            for staticlandmarks in data['staticlandmarks']:
                self.hdmap[staticlandmarks['id']] = staticlandmarks['position']
        except Exception as e:
            rospy.loginfo(f"ERROR: {e}")
        self.publishTopics()

    def publishTopics(self):
        # TODO: No rotation to calculate future bearing
        while not rospy.is_shutdown():
            landmarkArrMsg = ObjectBearingArrayStamped()
            t=0
            if self.starttime == None:
                self.starttime = rospy.Time.now()
                self.currenttime = self.starttime
                t=0
            else:
                self.currenttime = rospy.Time.now()
                # t= (self.currenttime - self.starttime).to_sec()
                t = 0

            for idx, landmarkInfo in self.hdmap.items():
                landmarkArrMsg.header.stamp = self.currenttime
                landmarkArrMsg.header.frame_id = "camera"
                landmarkMsg = ObjectBearing()
                pose = self.poseFormula(t)
                landmarkMsg.id = idx
                pij = pose.T - np.array(landmarkInfo)
                pij_norm = pij/np.linalg.norm(pij)
                direction = np.matmul(self.initRot, pij_norm.T).reshape((-1,))
                landmarkMsg.bearing_x = direction[0]
                landmarkMsg.bearing_y = direction[1]
                landmarkMsg.bearing_z = direction[2]
                landmarkArrMsg.bearing.append(deepcopy(landmarkMsg))
            # self.landmarkPub.publish(landmarkArrMsg)
            
            twistMsg = TwistStamped()
            twistMsg.header.stamp = self.currenttime
            twistMsg.header.frame_id = "base_link"
            # twistMsg.twist.twist.linear = Vector3(*self.linVelFormula(t))
            # twistMsg.twist.twist.angular = Vector3(*self.angVelFormula(t))
            twistMsg.twist.linear = Vector3(*[0, 0, 0])
            twistMsg.twist.angular = Vector3(*[0, 0, 0])

            self.twistPub.publish(twistMsg)

            msg = TransformStamped()
            msg.header.stamp = self.currenttime
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_link"
            msg.transform.translation = Vector3(*pose)
            msg.transform.rotation = Quaternion(*[0,0,0,1])
            self.br.sendTransform(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    riccati_occlusion_simulated_topics().run()