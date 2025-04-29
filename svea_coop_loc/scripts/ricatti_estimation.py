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

WITH_LANDMARK = True
MAP_FRAME = "map"
VISUALIZATION_MODE = False
SUFFIX = "2"
ERROR_MSG = {-1:"No error",
             0: "Not enough source points",
             1: "Algined source points",
             2: "Three non-aligned source points: moving along one of the straight lines of the danger cylinder and passing through a source point",
             3: "Three non-aligned source points: motionless C in danger cylinder",
             4: "Three non-aligned source points: moving on danger cylinder but not along any lines (weak)",
             5: "Four + non-aligned source points: on horopter curve",
            }

class riccati_estimation():
    def __init__(self):
        rospy.init_node('riccati_estimation')

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        ##############################################
        ################# Variables ##################
        self.timeStamp = None
        self.startTime = None
        self.svea_frame_name = "base_link"
        self.seq = 0
        self.t = 0
        
        self.camera_to_base_transform = np.array([None])
        self.ChangeVelocityRotMatrix = np.array([None])
        self.transBaseCamPose = np.array([None])

        while not isinstance(self.camera_to_base_transform, PoseStamped) or not isinstance(self.transBaseCamPose, PoseStamped) or not self.ChangeVelocityRotMatrix.any():
            self.GetStaticTransform()
            if rospy.is_shutdown():
                break
        self.CameraInfo = None

        # for debug
        self.stop = 0
        ################# Variables ##################
        ##############################################

        ##############################################
        ################# Publisher ##################
        self.dtPublisher = rospy.Publisher("/riccati/dt" + SUFFIX, Float32, queue_size=10)
        self.RiccatiSetupPublisher = rospy.Publisher("/riccati/setup" + SUFFIX, riccati_setup, queue_size=10)
        self.RiccatiDirPublisher = rospy.Publisher("/riccati/diretions" + SUFFIX , PoseArray, queue_size=10)

        self.usedLandmark = rospy.Publisher('/aruco/used' + SUFFIX, ArucoArray, queue_size=10)
        self.debugTopic = rospy.Publisher('/riccati/debug'+ SUFFIX, TransformStamped, queue_size=1)
        self.odomPub = rospy.Publisher('/riccati_localization/base_link/pose'+ SUFFIX, Odometry, queue_size=1)
        self.PoseCameraPub = rospy.Publisher('/riccati_localization/camera/pose'+ SUFFIX, PoseWithCovarianceStamped, queue_size=1)
        self.AngVelPub = rospy.Publisher('/riccati/angular_vel'+ SUFFIX, Vector3Stamped, queue_size=1)

        self.visualArucoPub = rospy.Publisher("/visual/cameraaruco"+ SUFFIX, VMA, queue_size=10)

        self.setEKFPose = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1) 

        ################# Publisher ##################
        ##############################################

        ################################### ###########
        ################# Parameters #################
        k = rospy.get_param("~k", 80)
        q = rospy.get_param("~q", 25)
        v1 = rospy.get_param("~v1", 1.0)
        v2 = rospy.get_param("~v2", 10.0)

        self.estpose = np.array([3.0, 0.5, 0], dtype=np.float64)
        self.estori = np.array([-0.001, 0, 0, 2], dtype=np.float64) #w, x, y, z #base_link
        self.initpose = self.estpose
        self.estori /= np.linalg.norm(self.estori)
        self.initori = self.estori #w, x, y, z
        ################# Parameters #################
        ##############################################
        self.riccati_obj = riccati_observer(
        stepsize                = 0.1,
        tol                     = 0.015, #1e-2 * 1, #1e-2 * 3,
        which_eq                = 0,
        p_hat                   = self.estpose, # sth from state or just input from lanuch file,
        Lambda_bar_0            = self.estori, #np.hstack((self.estori[-1], self.estori[0:-1])), # sth from state or just input from lanuch file,  # quaternion: w, x, y, z
        z_appear                = np.array([]),
        k                       = k,
        q                       = q, 
        v                       = np.array([v1, v2]),
        p_riccati               = np.array([1, 100])
        )
        ##############################################
        ################# Subscriber #################

        Twist = Subscriber('/actuation_twist', TwistWithCovarianceStamped)
        EKFOdom = rospy.Subscriber('/odometry/filtered/global', Odometry, self.odomEKFcallback)
        # Twist = Subscriber('/odometry/filtered', Odometry)
        
        Landmark = Subscriber('/aruco/detection/more', ArucoArray)

        LandmarkGroudtruth = Subscriber('/aruco/detection/Groundtruth', ArucoArray)
        self.groundtruthLandmarkSub = rospy.Subscriber('/aruco/detection/Groundtruth', ArucoArray, self.GroundtruthCallback)

        if WITH_LANDMARK:
            # sync = ApproximateTimeSynchronizer([Twist, Landmark], queue_size=16, slop=0.8) #maybe????, but should only apply to cases with changing velocity
            sync = ApproximateTimeSynchronizer([Twist, Landmark], queue_size=20, slop=0.5) #maybe????, but should only apply to cases with changing velocity
            # sync = ApproximateTimeSynchronizer([Twist, Landmark, LandmarkGroudtruth], queue_size=1, slop=0.5) #maybe????, but should only apply to cases with changing velocity
            sync.registerCallback(self.TwistAndLandmarkCallback)
        else:
            sync = ApproximateTimeSynchronizer([Twist], queue_size=1, slop=0.2)
            sync.registerCallback(self.TwistSyncCallback)

        self.cameraSub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.CameraInfoCallback)
        ################# Subscriber #################
        ##############################################
        
        self.pubinit = False

        self.EKFOdomtime = None
        self.EKFOdompose = None
        self.EKFOdomori = None
        self.LostCount = 0
        self.groundtruthLandmarkDictionary = {
        }
        
        # self.pub_EstPose(rospy.Time.now(), 0)
        self.changeFrame(rospy.Time.now())
    def GetStaticTransform(self):
        try:
            transCamBase = self.buffer.lookup_transform(self.svea_frame_name, "camera", rospy.Time(), rospy.Duration(1)) #frame id = svea5, child = camera 
            self.camera_to_base_transform = PoseStamped()
            self.camera_to_base_transform.pose.position = transCamBase.transform.translation
            self.camera_to_base_transform.pose.orientation = transCamBase.transform.rotation
            # print(self.camera_to_base_transform)
        except Exception as e:
            print(f"/ricatti_estimation/GetStaticTransform: {e}")

        try:
            transBaseCam = self.buffer.lookup_transform('camera', self.svea_frame_name, rospy.Time(), rospy.Duration(1))
            self.transBaseCamPose = PoseStamped()
            self.transBaseCamPose.pose.position = transBaseCam.transform.translation
            self.transBaseCamPose.pose.orientation = transBaseCam.transform.rotation
            self.ChangeVelocityRotMatrix = tf.transformations.quaternion_matrix([transBaseCam.transform.rotation.x, transBaseCam.transform.rotation.y, transBaseCam.transform.rotation.z, transBaseCam.transform.rotation.w])[:3, :3]
        except Exception as e:
            print(f"/ricatti_estimation/GetStaticTransform: {e}")

    def GroundtruthCallback(self ,LandmarkGroudtruthMsg):
        for ArucoGroundtruth in LandmarkGroudtruthMsg.arucos:
            self.groundtruthLandmarkDictionary[ArucoGroundtruth.marker.id] = [ArucoGroundtruth.marker.pose.pose.position.x, ArucoGroundtruth.marker.pose.pose.position.y, ArucoGroundtruth.marker.pose.pose.position.z]
        self.groundtruthLandmarkSub.unregister()

    def CameraInfoCallback(self, msg):
        self.CameraInfo = msg
        rospy.loginfo(f"Get camera info")
        self.cameraSub.unregister()

    def changeFrame(self, timeStamp):
        # pose = np.matmul(self.riccati_obj.rodrigues_formula(self.estori), self.estpose) #baselink in map frame
        msg2 = TransformStamped()
        msg2.header.seq = self.seq
        msg2.header.stamp = timeStamp #+ rospy.Duration(dt)
        msg2.header.frame_id = MAP_FRAME
        msg2.child_frame_id = "base_link_est" + SUFFIX

        msg2.transform.translation.x = self.estpose[0]
        msg2.transform.translation.y = self.estpose[1]
        msg2.transform.translation.z = self.estpose[2]
        msg2.transform.rotation.w = self.estori[0]
        msg2.transform.rotation.x = self.estori[1]
        msg2.transform.rotation.y = self.estori[2]
        msg2.transform.rotation.z = self.estori[3]
        self.sbr.sendTransform(msg2)
        msg2.child_frame_id = 'base_link'
        transformed_direction = tf2_geometry_msgs.do_transform_pose(self.camera_to_base_transform, msg2) 
        msg = TransformStamped()
        msg.header.seq = self.seq
        msg.header.stamp = timeStamp #+ rospy.Duration(dt)
        msg.header.frame_id = MAP_FRAME
        msg.child_frame_id = "camera_est" + SUFFIX

        msg.transform.translation = transformed_direction.pose.position
        msg.transform.rotation = transformed_direction.pose.orientation

        self.estori = np.array([transformed_direction.pose.orientation.w, transformed_direction.pose.orientation.x, transformed_direction.pose.orientation.y, transformed_direction.pose.orientation.z])
        self.estpose = np.array([transformed_direction.pose.position.x, transformed_direction.pose.position.y, transformed_direction.pose.position.z])
        # pose = np.matmul(np.transpose(self.riccati_obj.rodrigues_formula(self.estori)), self.estpose)
        # self.estpose = pose
        self.riccati_obj.set_init(self.estori, self.estpose)
        self.sbr.sendTransform(msg)

        rospy.loginfo(f"Set up Initial Estimation")

        self.seq += 1
        self.pubinit = True

    def odomEKFcallback(self, msg):
        self.EKFOdomtime = msg.header.stamp
        self.EKFOdompose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.EKFOdomori = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]

    def publishTOEKFodom(self, odommsg):
        msg = PoseWithCovarianceStamped()
        msg.header = odommsg.header
        msg.pose.pose = odommsg.pose.pose
        cov_matrix = [1e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 1e-6, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 1e-6, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 1e-6]
        msg.pose.covariance = cov_matrix
        self.setEKFPose.publish(msg)

    def pub_EstPose(self, timeStamp, dt, detect_landmark):
        # print("pub_EstPose")
        pose = np.matmul(self.riccati_obj.rodrigues_formula(self.estori), self.estpose) #baselink in map frame

        msg2 = TransformStamped()
        msg2.header.seq = self.seq
        msg2.header.stamp = timeStamp + rospy.Duration(dt)
        msg2.header.frame_id = MAP_FRAME
        msg2.child_frame_id = 'camera_est' + SUFFIX

        msg2.transform.translation.x = pose[0]
        msg2.transform.translation.y = pose[1]
        msg2.transform.translation.z = pose[2]
        msg2.transform.rotation.w = self.estori[0]
        msg2.transform.rotation.x = self.estori[1]
        msg2.transform.rotation.y = self.estori[2]
        msg2.transform.rotation.z = self.estori[3]

        self.sbr.sendTransform(msg2)
        # print(msg2)
        cammsg = PoseWithCovarianceStamped()
        cammsg.header = msg2.header
        cammsg.header.frame_id = "camera"
        cammsg.pose.pose.position.x = np.round(pose[0], 4)
        cammsg.pose.pose.position.y = np.round(pose[1], 4)
        cammsg.pose.pose.position.z = np.round(pose[2], 4)

        cammsg.pose.pose.orientation.w = np.round(self.estori[0], 4)
        cammsg.pose.pose.orientation.x = np.round(self.estori[1], 4)
        cammsg.pose.pose.orientation.y = np.round(self.estori[2], 4)
        cammsg.pose.pose.orientation.z = np.round(self.estori[3], 4)
        cammsg.pose.covariance = [0.001] * 36

        msg2.child_frame_id = 'camera'
        transformed_direction = tf2_geometry_msgs.do_transform_pose(self.transBaseCamPose, msg2) 

        ##################################################

        msg = TransformStamped()
        msg.header.seq = self.seq
        msg.header.stamp = timeStamp + rospy.Duration(dt)
        msg.header.frame_id = MAP_FRAME 
        msg.child_frame_id = 'base_link_est' + SUFFIX

        msg.transform.translation = transformed_direction.pose.position
        msg.transform.rotation = transformed_direction.pose.orientation
        self.sbr.sendTransform(msg)   

        odommsg = Odometry()
        odommsg.header = msg.header
        msg.header.frame_id = MAP_FRAME 
        odommsg.child_frame_id = "base_link"
        odommsg.pose.pose.position.x = transformed_direction.pose.position.x
        odommsg.pose.pose.position.y = transformed_direction.pose.position.y
        odommsg.pose.pose.position.z = transformed_direction.pose.position.z
        odommsg.pose.pose.orientation.w = transformed_direction.pose.orientation.w
        odommsg.pose.pose.orientation.x = transformed_direction.pose.orientation.x
        odommsg.pose.pose.orientation.y = transformed_direction.pose.orientation.y
        odommsg.pose.pose.orientation.z = transformed_direction.pose.orientation.z
        odommsg.pose.covariance = [0.0001] * 36


        self.odomPub.publish(odommsg)
        self.PoseCameraPub.publish(cammsg)

        # if detect_landmark:
            # if transformed_direction.pose.position.z < 0.5 and transformed_direction.pose.position.z > -0.5:
            # self.odomPub.publish(odommsg)
            # self.PoseCameraPub.publish(cammsg)
        # if not detect_landmark:
            # if self.LostCount >=5:
                # self.estori = self.EKFOdomori
                # self.estpose = self.EKFOdompose
                # self.changeFrame(self.EKFOdomtime)
                # self.riccati_obj.set_init(self.estori, self.estpose)
                # self.LostCount = 0
            # else:
                # self.odomPub.publish(odommsg)
                # self.PoseCameraPub.publish(cammsg)
                # self.LostCount += 1

        self.debugTopic.publish(msg)

        self.seq += 1

        visualarr = VMA()
        visualmarker = VM()
        visualmarker.header.stamp = timeStamp
        visualmarker.header.frame_id = MAP_FRAME
        visualmarker.ns = "sveapose"
        visualmarker.id = 0
        visualmarker.type = VM.ARROW
        visualmarker.action = VM.ADD
        visualmarker.pose.position.x = transformed_direction.pose.position.x
        visualmarker.pose.position.y = transformed_direction.pose.position.y
        visualmarker.pose.position.z = transformed_direction.pose.position.z
        visualmarker.pose.orientation.w = transformed_direction.pose.orientation.w
        visualmarker.pose.orientation.x = transformed_direction.pose.orientation.x
        visualmarker.pose.orientation.y = transformed_direction.pose.orientation.y
        visualmarker.pose.orientation.z = transformed_direction.pose.orientation.z
        visualmarker.scale = Vector3(*[0.2, 0.2, 0.2])
        visualmarker.color = ColorRGBA(*[255.0/450.0, 92.0/450.0, 103.0/450.0, 1.0])
        visualarr.markers.append(visualmarker)
        self.visualArucoPub.publish(visualarr)

    def pubRiccatiMsg(self):
        riccatiMsg = riccati_setup()
        riccatiMsg.stepsize = self.riccati_obj.stepsize
        riccatiMsg.tol = self.riccati_obj.tol
        riccatiMsg.which_eq = self.riccati_obj.which_eq
        riccatiMsg.k = self.riccati_obj.k
        riccatiMsg.q = self.riccati_obj.q
        riccatiMsg.v = self.riccati_obj.v
        riccatiMsg.pose.position = Point(*self.initpose)
        riccatiMsg.pose.orientation.w = self.initori[0]
        riccatiMsg.pose.orientation.x = self.initori[1]
        riccatiMsg.pose.orientation.y = self.initori[2]
        riccatiMsg.pose.orientation.z = self.initori[3]

        self.RiccatiSetupPublisher.publish(riccatiMsg)

    def visualizeAruco(self, ArucoList, landmark):
        visualarr = VMA()
        for idx, aruco in enumerate(ArucoList.arucos):
            visualmarker = VM()
            visualmarker.header = aruco.marker.header
            visualmarker.header.frame_id = self.svea_frame_name
            visualmarker.ns = "cameraAruco"
            visualmarker.id = aruco.marker.id
            visualmarker.type = VM.SPHERE
            visualmarker.action = VM.ADD
            visualmarker.pose.position = Point(*landmark[idx])
            visualmarker.pose.orientation = Quaternion(*[0, 0, 0, 1])
            visualmarker.scale = Vector3(*[0.2, 0.2, 0.2])
            visualmarker.color = ColorRGBA(*[1.0, 0.0, 0.0, 1.0])
            visualarr.markers.append(visualmarker)
        self.visualArucoPub.publish(visualarr)
    
    def visualDirection(self, z, msg, arucoId):
        dirs = self.riccati_obj.calculate_direction(z)
        Pmsg = PoseArray()
        Pmsg.header = msg.header

        for landmark_idx in range(len(z)):
            d = dirs[landmark_idx]
            Tmsg = TransformStamped()
            Tmsg.header = msg.header
            Tmsg.header.frame_id = 'camera_est' + SUFFIX
            Tmsg.child_frame_id = 'Direction' + str(arucoId[landmark_idx])
            Tmsg.transform.translation = Vector3(*d)
            Tmsg.transform.rotation = Quaternion(*[0, 0, 0, 1]) #x, y, z, w
            self.sbr.sendTransform(Tmsg)
            posemsg = Pose()
            posemsg.position = Point(*d)
            posemsg.orientation = Quaternion(*[0, 0, 0, 1])
            Pmsg.poses.append(posemsg)
        self.RiccatiDirPublisher.publish(Pmsg)

    # def TwistAndLandmarkCallback(self, TwistMsg, LandmarkMsg, LandmarkGroudtruthMsg):
    def TwistAndLandmarkCallback(self, TwistMsg, LandmarkMsg):
        # if self.stop <1:
        # rospy.loginfo(f"self.camera_to_base_transform {self.camera_to_base_transform != None}, self.CameraInfo {self.CameraInfo != None}, self.pubinit {self.pubinit}")
        if self.camera_to_base_transform != None and self.CameraInfo != None and self.pubinit:
            if self.startTime == None:
                self.startTime = LandmarkMsg.header.stamp
            self.pubRiccatiMsg()
            self.timeStamp = LandmarkMsg.header.stamp
            
            linear_velocity = np.array([TwistMsg.twist.twist.linear.x, TwistMsg.twist.twist.linear.y, TwistMsg.twist.twist.linear.z])
            linear_velocity = np.dot(self.ChangeVelocityRotMatrix, linear_velocity)    
            angular_velocity = np.array([TwistMsg.twist.twist.angular.x, TwistMsg.twist.twist.angular.y, TwistMsg.twist.twist.angular.z])
            angular_velocity = np.dot(self.ChangeVelocityRotMatrix, angular_velocity)    
            # print("linear_velocity ", linear_velocity)
            # print("angular_velocity ", angular_velocity)
            angmsg = Vector3Stamped()
            angmsg.header = TwistMsg.header
            angmsg.vector.x = angular_velocity[0]
            angmsg.vector.y = angular_velocity[1]
            angmsg.vector.z = angular_velocity[2]
            self.AngVelPub.publish(angmsg)

            landmark = []
            landmarkGroundTruth = []
            arucosUsed = ArucoArray()
            arucosUsed.header = LandmarkMsg.header
            arucoId = []

            for aruco in LandmarkMsg.arucos:
                # if aruco.marker.id != 37:
                temp = np.array([aruco.image_x, aruco.image_y, 1])
                temp /= np.linalg.norm(temp)
                landmark.append(temp)
                arucosUsed.arucos.append(aruco)
                arucoId.append(aruco.marker.id)
                landmarkGroundTruth.append(self.groundtruthLandmarkDictionary[aruco.marker.id])
                # for ArucoGroundtruth in LandmarkGroudtruthMsg.arucos:
                    # if ArucoGroundtruth.marker.id == aruco.marker.id:
                        # in map frame  
                        # landmarkGroundTruth.append([ArucoGroundtruth.marker.pose.pose.position.x, ArucoGroundtruth.marker.pose.pose.position.y, ArucoGroundtruth.marker.pose.pose.position.z])
                        # break
            current_time = (self.timeStamp - self.startTime).to_sec()
            detect_landmark = self.riccati_obj.update_measurement(angular_velocity, linear_velocity, landmark, landmarkGroundTruth, current_time)
            if detect_landmark:
                self.usedLandmark.publish(arucosUsed)
            else:
                arucosUsed.arucos = []
                self.usedLandmark.publish(arucosUsed)
            if VISUALIZATION_MODE:
                self.visualizeAruco(arucosUsed, landmark)
            self.visualDirection(landmark, LandmarkMsg, arucoId)
            
            solt, dt, soly, errMsg = self.riccati_obj.step_simulation()
            dtMsg = Float32()
            dtMsg.data = dt
            self.dtPublisher.publish(dtMsg)
            self.t = solt

            ############################# Quaternion
            ############################# 
            self.estpose = soly[4:7]
            # print("self.estpose", self.estpose)
            self.estori = soly[0:4]
            self.estori /= np.linalg.norm(self.estori)
            ############################# Quaternion
            #############################

            ############################# 
            ############################# Rot Mat        
            # self.estpose = soly[9:12]
            # Rotmat = soly[0:9].reshape((3,3))
            # Translmat = soly[9:12].reshape((3,1))
            # Transmat = np.hstack((Rotmat, Translmat))
            # Transmat = np.vstack((Transmat, np.array([0,0,0,1])))
            # self.estori = tf.transformations.quaternion_from_matrix(Transmat) 
            # self.estori = np.concatenate(([self.estori[-1]], self.estori[0:-1]))
            # self.estori /= np.linalg.norm(self.estori)
            ############################# Rot Mat        
            #############################        
            self.pub_EstPose(self.timeStamp, dt, detect_landmark)
            # rospy.loginfo(f'current_time, {current_time}, {dt}')
            # rospy.loginfo(f"===================================================")
            self.stop += 1


    def TwistSyncCallback(self, TwistMsg):
        if self.startTime == None:
            self.startTime = TwistMsg.header.stamp
        self.pubRiccatiMsg()
        self.timeStamp = TwistMsg.header.stamp

        linear_velocity = np.array([TwistMsg.twist.twist.linear.x, TwistMsg.twist.twist.linear.y, TwistMsg.twist.twist.linear.z])
        linear_velocity = np.dot(self.ChangeVelocityRotMatrix, linear_velocity)    
        angular_velocity = np.array([TwistMsg.twist.twist.angular.x, TwistMsg.twist.twist.angular.y, TwistMsg.twist.twist.angular.z])
        angular_velocity = np.dot(self.ChangeVelocityRotMatrix, angular_velocity)  
        angmsg = Vector3Stamped()
        angmsg.header = TwistMsg.header
        angmsg.vector.x = angular_velocity[0]
        angmsg.vector.y = angular_velocity[1]
        angmsg.vector.z = angular_velocity[2]
        self.AngVelPub.publish(angmsg)
        # print("linear_velocity ", linear_velocity)
        # print("angular_velocity ", angular_velocity)

        detect_landmark = self.riccati_obj.update_measurement(angular_velocity, linear_velocity, [], [], (self.timeStamp - self.startTime).to_sec())
        solt, dt, soly, errMsg = self.riccati_obj.step_simulation()
        # rospy.loginfo(f'errMsg: {ERROR_MSG[errMsg]}')
        dtMsg = Float32()
        dtMsg.data = dt
        self.dtPublisher.publish(dtMsg)
        self.t = solt

        ############################# Quaternion
        ############################# 
        self.estpose = soly[4:7]
        print("self.estpose", self.estpose)
        self.estori = soly[0:4]
        self.estori /= np.linalg.norm(self.estori)
        ############################# Quaternion
        #############################

        ############################# 
        ############################# Rot Mat        
        # self.estpose = soly[9:12]
        # Rotmat = soly[0:9].reshape((3,3))
        # Translmat = soly[9:12].reshape((3,1))
        # Transmat = np.hstack((Rotmat, Translmat))
        # Transmat = np.vstack((Transmat, np.array([0,0,0,1])))
        # self.estori = tf.transformations.quaternion_from_matrix(Transmat) 
        # self.estori = np.concatenate(([self.estori[-1]], self.estori[0:-1]))
        # self.estori /= np.linalg.norm(self.estori)
        ############################# Rot Mat        
        #############################        
        self.pub_EstPose(self.timeStamp, dt, detect_landmark)
        print("===================================================")
        self.stop += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    current_pid = os.getpid()
    process = psutil.Process(current_pid)
    process.cpu_affinity([2,3,4])
    riccati_estimation().run()