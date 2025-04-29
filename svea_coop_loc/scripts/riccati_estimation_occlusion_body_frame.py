#! /usr/bin/env python3
import psutil
import os

import rospy
import yaml

import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

from riccati_observer_occlusion_body_frame import riccati_observer_occlusion

from svea_thesis.msg import riccati_setup
from svea_thesis.msg import envSetting, agentSetting
from pixy_camera.msg import ObjectBearing, ObjectBearingArrayStamped
from svea_thesis.srv import GetPosition, GetPositionResponse

from std_msgs.msg import Float32, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, Point, PointStamped, Quaternion, Vector3, PoseStamped, Vector3Stamped, TwistStamped, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu

import tf2_ros
import tf2_geometry_msgs
import tf.transformations

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from copy import deepcopy
class riccati_estimation_occlusion():
    def __init__(self):
        rospy.init_node('riccati_estimation_occlusion')
        self.DEBUG = rospy.get_param('~debug', False)

        # test
        self.vehicleAgentTime = None

        self.NAMESPACE = rospy.get_namespace()
        self.SVEANAME = None
        self.vehicleAgents = {}
        self.vehicleAgentsPose = {}

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        # Frame
        self.MAPFRAME = rospy.get_param('~mapframe', "map")
        self.baseFrame = rospy.get_param("~baseframe", "base_link")
        self.cameraFrame = rospy.get_param("~cameraframe", "camera")
    
        self.CamBaseTransform = PoseStamped()
        self.BaseCamTransform = PoseStamped()
        self.transBaseCamVelocityRot = None
        
        # Initialization
        self.DoneInitialization = 0

        # OFFLINE RUN
        self.SUFFIX = rospy.get_param('~suffix', "")

        # Varibles - Time
        self.timeStamp = rospy.Time.now()
        self.startTime = None
        self.dt = 0

        # Variables
        k = rospy.get_param('~k', 0)
        q = rospy.get_param('~q', 0)
        v1 = rospy.get_param('~v1', 0)
        v2 = rospy.get_param('~v2', 0)

        self.estpose = rospy.get_param('~pose')
        self.estpose = np.array(self.estpose, dtype=np.float64)
        self.estori = rospy.get_param('~orientation')
        self.estori = np.array(self.estori, dtype=np.float64) #quaternion: w, x, y, z
        self.estori /= np.linalg.norm(self.estori)

        self.initpose = deepcopy(self.estpose)
        self.initori = deepcopy(self.estori)

        self.landmarks = {} # type, position, bearing
        # type: {0 = leader, 1 = vehicle} # just to distinguish when to obtain pose data??

        self.riccati_obj = riccati_observer_occlusion(
            stepsize                = 0.0001,
            tol                     = (1e-3)/2,
            p_hat                   = self.estpose, 
            Lambda_hat              = self.estori, # quaternion: w, x, y, z
            k                       = k,
            q                       = q, 
            v                       = np.array([v1, v2]),
            p_riccati               = np.array([1, 10])
        )
        
        # Publishers
        self.RiccatiSetupPublisher = rospy.Publisher('~riccati/setup' + str(self.SUFFIX), riccati_setup, queue_size=10, latch=True)
        self.VishdMapPub = rospy.Publisher('~visualize/hdMap' + str(self.SUFFIX), VMA, queue_size=10, latch=True)
        self.VisPosePub = rospy.Publisher('~visualize/riccatiPose' + str(self.SUFFIX), VMA, queue_size=10)
        self.VisDetected = rospy.Publisher('~visualize/detected' + str(self.SUFFIX), VMA, queue_size=10)
        self.EnvPub = rospy.Publisher('~riccati/envSetup' + str(self.SUFFIX), envSetting, queue_size=10, latch=True)
        self.pointPub = rospy.Publisher(f'{self.NAMESPACE}riccatiPoint' + str(self.SUFFIX), PointStamped, queue_size=10)

        # For NATS delay
        self.LandmarkLeadertimePub = rospy.Publisher('landmark_leader_diff', Float32, queue_size=10)
        self.LandmarkCurrenttimePub = rospy.Publisher('landmark_current_diff', Float32, queue_size=10)

        self.hdmap = {} #id, position 
        setting_yaml_file_path = rospy.get_param('~env_yaml', "")
        communication = None
        if not setting_yaml_file_path == "":
            communication = self.init_env(setting_yaml_file_path)
        else:
            # TODO: add something to get landmark from 
            pass
        
        # Services - Servers
        # rospy.Service(f'{self.NAMESPACE}getPositionSrv', GetPosition, self.handleGetPosition)

        # Initialization
        self.GetStaticTransform()
        self.sendInitTransPose()

        # Services - Clients
        # TODO: maybe something that allow user to set the number of vehicles (might make more sense to do wait for service right before calling it)
        # for idx in range(1, 7):
            # if idx not in self.hdmap:
                # try:
                    # rospy.loginfo(f"waiting for service agent{idx}/getPositionSrv")
                    # rospy.wait_for_service(f'/agent{idx}/getPositionSrv', timeout=1)
                # except:
                    # rospy.loginfo(f"No service for agent{idx}/getPositionSrv")

        # Subscribers
        # cameraInfo_topic = rospy.get_param('~cameraInfo_topic', '/pixy_camera/camera_info')
        # rospy.rospy.Subscriber(cameraInfo_topic, CameraInfo, self.CameraInfoCallback)
        self.vehicleSubscription(communication)

        landmarks_topic = rospy.get_param('~landmarks_topic', self.NAMESPACE + 'pixy_landmark_detection/objectsBearing')
        landmarks_sub = Subscriber(landmarks_topic, ObjectBearingArrayStamped)

        twist_topic = rospy.get_param('~twist_topic', self.NAMESPACE + 'actuation_twist')
        twist_sub = Subscriber(twist_topic, TwistWithCovarianceStamped)

        sync_topics = ApproximateTimeSynchronizer([landmarks_sub, twist_sub], queue_size=1, slop=0.03)
        sync_topics.registerCallback(self.measurementCallback)

        self.DoneInitialization += 1


############################################# INITIALIZATION ##############################################
    def init_env(self, yaml_file_path):
        rospy.loginfo("Setting up environment")
        # TODO: would be nice to color code the markers in foxglove
        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"Failed to load Env from YAML, {e}")
        AllTransforms = []
        VisArrMsg = VMA()
        for staticlandmarks in data['staticlandmarks']:
            self.hdmap[staticlandmarks['id']] = staticlandmarks['position']

            landmarkTransformMsg = TransformStamped()
            landmarkTransformMsg.header.stamp = rospy.Time.now()
            landmarkTransformMsg.header.seq = 1
            landmarkTransformMsg.header.frame_id = "map"
            landmarkTransformMsg.child_frame_id = "Landmark" + str(staticlandmarks['id'])
            landmarkTransformMsg.transform.translation = Vector3(*staticlandmarks['position'])
            landmarkTransformMsg.transform.rotation = Quaternion(*[0,0,0,1])
            AllTransforms.append(landmarkTransformMsg)

            VisMarker = VM()
            VisMarker.header.stamp = self.timeStamp
            VisMarker.header.frame_id = self.MAPFRAME
            VisMarker.ns = "HDMap"
            VisMarker.id = staticlandmarks['id']
            VisMarker.type = VM.CUBE
            VisMarker.action = VM.ADD
            VisMarker.pose.position = Vector3(*staticlandmarks['position'])
            VisMarker.pose.orientation = Quaternion(*[0,0,0,1])
            VisMarker.scale = Vector3(*[0.2, 0.2, 0.2])
            VisMarker.color = ColorRGBA(*[50.0/450.0, 250.0/450.0, 50.0/450.0, 1.0])
            VisArrMsg.markers.append(VisMarker)

        self.sbr.sendTransform(AllTransforms)
        self.VishdMapPub.publish(VisArrMsg)

        envMsg = envSetting()
        for agent in data['agents']:
            agentMsg = agentSetting()
            agentMsg.AgentIdx = agent['idx']
            agentMsg.vehicleType = agent['vehicleType']
            agentMsg.sveaId = agent['sveaId']
            self.vehicleAgents[agent['idx']] = {"sveaId": "svea" + str(agent['sveaId'])}
            self.vehicleAgentsPose["svea" + str(agent['sveaId'])] = np.array([None, None, None], dtype=np.float64)
            envMsg.agents.append(agentMsg)
            if self.NAMESPACE == "/agent" + str(agent['idx']) + "/":
                self.SVEANAME = "svea" + str(agent['sveaId'])
        for env in data['env']:
            envMsg.communicationMethod = env['communicationMethod']

        self.EnvPub.publish(envMsg)
        self.DoneInitialization += 1
        rospy.loginfo("Environment setup completed")
        if "qualisys" in env['communicationMethod']:
            self.communicationMethod = "qualisys"
            return "qualisys"
        elif "NATS" in env['communicationMethod']:
            return "NATS"
        else:
            rospy.logwarn(f"Did not specify communication method, default NATS")
            return "NATS"

    def publish_RiccatiMsg(self):
        riccatiMsg = riccati_setup()
        riccatiMsg.tol = self.riccati_obj.tol
        riccatiMsg.k = self.riccati_obj.k
        riccatiMsg.q = self.riccati_obj.q
        riccatiMsg.v = self.riccati_obj.v
        riccatiMsg.pose.position = Point(*self.initpose)
        riccatiMsg.pose.orientation.w = self.initori[0]
        riccatiMsg.pose.orientation.x = self.initori[1]
        riccatiMsg.pose.orientation.y = self.initori[2]
        riccatiMsg.pose.orientation.z = self.initori[3]

        self.RiccatiSetupPublisher.publish(riccatiMsg)

    def vehicleSubscription(self, communication):
        if "qualisys" in communication:
            for idx, item in self.vehicleAgents.items():
                rospy.Subscriber("/qualisys/" + item["sveaId"] + "/pose", PoseStamped, self.qualisysCallback, callback_args=item["sveaId"])

        else:
            for idx, item in self.vehicleAgents.items():
                rospy.Subscriber("/agent" + str(idx) + "/riccatiPoint", PointStamped, self.Pointcallback, callback_args=idx)
            
############################################# INITIALIZATION ##############################################

############################################# VISUALIZATION ##############################################
    def visualizePose(self, transformed_direction):
        VisArrMsg = VMA()
        VisMarker = VM()
        VisMarker.header.stamp = self.timeStamp
        VisMarker.header.frame_id = self.MAPFRAME
        VisMarker.ns = "RiccatiPose"
        VisMarker.id = 0
        VisMarker.type = VM.ARROW
        VisMarker.action = VM.ADD
        VisMarker.pose.position = transformed_direction.pose.position #Vector3(*self.estpose)
        VisMarker.pose.orientation = transformed_direction.pose.orientation #Quaternion(*self.estori)
        VisMarker.scale = Vector3(*[0.4, 0.2, 0.3])
        VisMarker.color = ColorRGBA(*[255.0/450.0, 92.0/450.0, 103.0/450.0, 1.0])
        VisArrMsg.markers.append(VisMarker)
        self.VisPosePub.publish(VisArrMsg)
        pointMsg = PointStamped()
        pointMsg.header = VisMarker.header
        pointMsg.point = transformed_direction.pose.position
        self.pointPub.publish(pointMsg)

    def visualizeBearing(self):
        AllTransforms = []
        msg = TransformStamped()
        msg.header.stamp = self.timeStamp
        msg.header.frame_id = self.cameraFrame
        msg.transform.rotation = Quaternion(*[0,0,0,1])
        for idx, pose in self.landmarks.items():
            msg.child_frame_id = 'direction' + str(idx)
            msg.transform.translation = Vector3(*pose["bearing"])
            AllTransforms.append(deepcopy(msg))
        self.sbr.sendTransform(AllTransforms)

    def visualizeDetectedVehicles(self):
        VisArrMsg = VMA()
        VisMarker = VM()
        VisMarker.header.stamp = self.timeStamp
        VisMarker.header.frame_id = self.MAPFRAME
        VisMarker.ns = "DetectedLandmarks"
        VisMarker.type = VM.SPHERE
        VisMarker.action = VM.DELETEALL
        VisArrMsg.markers.append(VisMarker)
        self.VisDetected.publish(VisArrMsg)
        VisArrMsg = VMA()
        for idx, landmark_info in self.landmarks.items():
            VisMarker = VM()
            VisMarker.header.stamp = self.timeStamp
            VisMarker.header.frame_id = self.MAPFRAME
            VisMarker.ns = "DetectedLandmarks"
            VisMarker.id = idx
            VisMarker.type = VM.SPHERE
            VisMarker.action = VM.ADD
            VisMarker.pose.position = Vector3(*landmark_info['position'])
            VisMarker.pose.orientation = Quaternion(*[0, 0, 0, 1])
            VisMarker.scale = Vector3(*[0.3, 0.3, 0.3])
            VisMarker.color = ColorRGBA(*[0.0, 0.0, 1.0, 1.0])
            VisArrMsg.markers.append(VisMarker)
        self.VisDetected.publish(VisArrMsg)
############################################# VISUALIZATION ##############################################

############################################# TRANSFORM ##############################################
    def GetStaticTransform(self):
        try:
            transCamBase = self.buffer.lookup_transform("base_link", "camera", rospy.Time(), rospy.Duration(1)) #frame id = svea5, child = camera 
            self.CamBaseTransform.pose.position = transCamBase.transform.translation
            self.CamBaseTransform.pose.orientation = transCamBase.transform.rotation
            # print(self.camera_to_base_transform)
        except Exception as e:
            print(f"/ricatti_estimation/GetStaticTransform: {e}")

        try:
            transBaseCam = self.buffer.lookup_transform('camera', "base_link", rospy.Time(), rospy.Duration(1))
            self.BaseCamTransform.pose.position = transBaseCam.transform.translation
            self.BaseCamTransform.pose.orientation = transBaseCam.transform.rotation
            self.transBaseCamVelocityRot = tf.transformations.quaternion_matrix([transBaseCam.transform.rotation.x, transBaseCam.transform.rotation.y, transBaseCam.transform.rotation.z, transBaseCam.transform.rotation.w])[:3, :3]
        except Exception as e:
            print(f"/ricatti_estimation/GetStaticTransform: {e}")

    def sendInitTransPose(self):
        AllTransforms = []
        msg = TransformStamped()
        msg.header.stamp = self.timeStamp
        msg.header.frame_id = self.MAPFRAME
        msg.child_frame_id = self.baseFrame + self.SUFFIX
        msg.transform.translation.x = self.estpose[0]
        msg.transform.translation.y = self.estpose[1]
        msg.transform.translation.z = self.estpose[2]
        msg.transform.rotation.w = self.estori[0]
        msg.transform.rotation.x = self.estori[1]
        msg.transform.rotation.y = self.estori[2]
        msg.transform.rotation.z = self.estori[3]
        AllTransforms.append(deepcopy(msg))

        msg.child_frame_id = 'base_link'
        transformed_direction = tf2_geometry_msgs.do_transform_pose(self.CamBaseTransform, msg) 
        
        msg.child_frame_id = self.cameraFrame + self.SUFFIX
        msg.transform.translation = transformed_direction.pose.position
        msg.transform.rotation = transformed_direction.pose.orientation

        self.estori = np.array([transformed_direction.pose.orientation.w, transformed_direction.pose.orientation.x, transformed_direction.pose.orientation.y, transformed_direction.pose.orientation.z])
        self.estpose = np.array([transformed_direction.pose.position.x, transformed_direction.pose.position.y, transformed_direction.pose.position.z])
        self.riccati_obj.set_init(self.estori, self.estpose)
        
        AllTransforms.append(msg)
        self.sbr.sendTransform(AllTransforms)
        self.publish_RiccatiMsg()
        self.DoneInitialization += 1

    # send transform pose after estimation
    def sendTransPose(self):
        pose = np.matmul(self.riccati_obj.rodrigues_formula(self.estori), self.estpose) #baselink in map frame

        AllTransforms = []
        msg = TransformStamped()
        msg.header.stamp = self.timeStamp + rospy.Duration(self.dt)
        msg.header.frame_id = self.MAPFRAME
        msg.child_frame_id = self.cameraFrame + self.SUFFIX
        msg.transform.translation.x = pose[0]
        msg.transform.translation.y = pose[1]
        msg.transform.translation.z = pose[2]
        msg.transform.rotation.w = self.estori[0]
        msg.transform.rotation.x = self.estori[1]
        msg.transform.rotation.y = self.estori[2]
        msg.transform.rotation.z = self.estori[3]
        AllTransforms.append(deepcopy(msg))

        cammsg = PoseWithCovarianceStamped()
        cammsg.header = msg.header
        cammsg.header.frame_id = "camera"
        cammsg.pose.pose.position = Vector3(*pose)
        cammsg.pose.pose.orientation.w = self.estori[0]
        cammsg.pose.pose.orientation.x = self.estori[1]
        cammsg.pose.pose.orientation.y = self.estori[2]
        cammsg.pose.pose.orientation.z = self.estori[3]
        cammsg.pose.covariance = [0.001] * 36

        msg.child_frame_id = 'camera'
        transformed_direction = tf2_geometry_msgs.do_transform_pose(self.BaseCamTransform, msg) 

        msg.header.frame_id = self.MAPFRAME
        msg.child_frame_id = self.baseFrame + self.SUFFIX
        msg.transform.translation = transformed_direction.pose.position
        msg.transform.rotation = transformed_direction.pose.orientation
        AllTransforms.append(msg)
        self.sbr.sendTransform(AllTransforms)   
        self.visualizePose(transformed_direction) 

############################################# TRANSFORM ##############################################


############################################# CALLBACKS ##############################################
    def Pointcallback(self, msg, whichAgent):
        #TODO: unregister this svea from vehicleAgentsPose
        self.vehicleAgentsPose[self.vehicleAgents[whichAgent]["sveaId"]] = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        self.vehicleAgentTime = msg.header.stamp

    def qualisysCallback(self, msg, whichSvea):
        #TODO: unregister this svea from vehicleAgentsPose
        self.vehicleAgentsPose[whichSvea] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        self.vehicleAgentTime = msg.header.stamp

    def measurementCallback(self, landmarkMsg, twistMsg):
        if self.DoneInitialization >= 3:
            if self.startTime == None:
                self.startTime = landmarkMsg.header.stamp
            self.timeStamp = landmarkMsg.header.stamp
            CURRENTTIME = rospy.Time.now()
            # TODO: error detection catching?
            self.landmarks = {}
            for bearing in landmarkMsg.bearing:
                if bearing.id in self.hdmap:
                    # these are landmarks
                    type = 0 
                    self.landmarks[bearing.id] = {'type': type, 
                                                'position': np.array(self.hdmap[bearing.id], dtype=np.float64), 
                                                'bearing': np.array([bearing.bearing_x, bearing.bearing_y, bearing.bearing_z])
                                                }
                else:
                    # these are vehicles (leader and followers)
                    type = 1
                    try:
                        ######################################## with NATS SERVICE ########################################
                        # temp_timer = rospy.Time.now()
                        # rospy.wait_for_service(f'/agent{bearing.id}/getPositionSrv', timeout=0.1)
                        # getPositionSrv = rospy.ServiceProxy(f'/agent{bearing.id}/getPositionSrv', GetPosition)
                        # response = getPositionSrv(True)
                        # print((rospy.Time.now()-temp_timer).to_sec())
                        # self.landmarks[bearing.id] = {'type': type, 
                                                    # 'position': np.array([response.pose_x, response.pose_y, response.pose_z]), 
                                                    # 'bearing': np.array([bearing.bearing_x, bearing.bearing_y, bearing.bearing_z])
                                                    # }
                        ######################################## with NATS SERVICE ########################################
                        
                        ######################################## with NATS PUBLISHER ########################################
                        whichSvea = self.vehicleAgents[bearing.id]["sveaId"]
                        if self.vehicleAgentsPose[whichSvea].any() != None:
                            self.LandmarkLeadertimePub.publish(Float32(data = (self.timeStamp - self.vehicleAgentTime).to_sec()))
                            self.landmarks[bearing.id] = {'type': type, 
                                                        'position': self.vehicleAgentsPose[whichSvea], 
                                                        'bearing': np.array([bearing.bearing_x, bearing.bearing_y, bearing.bearing_z])
                                                        }
                        else:
                            rospy.loginfo(f"Skipped {whichSvea}, cannot obtain its position")
                        self.LandmarkCurrenttimePub.publish(Float32(data = (self.timeStamp - CURRENTTIME).to_sec()))
                        ######################################## with NATS PUBLISHER ########################################
                        ######################################## with qualisys ########################################
                        # whichSvea = self.vehicleAgents[bearing.id]["sveaId"]
                        # self.landmarks[bearing.id] = {'type': type, 
                        #                             'position': self.vehicleAgentsPose[whichSvea], 
                        #                             'bearing': np.array([bearing.bearing_x, bearing.bearing_y, bearing.bearing_z])
                        #                             }
                        ######################################## with qualisys ########################################
                    except Exception as e:
                        rospy.logerr(f"Error: {e}")

            self.visualizeBearing()
            self.visualizeDetectedVehicles()

            linear_velocity = np.array([twistMsg.twist.twist.linear.x, twistMsg.twist.twist.linear.y, twistMsg.twist.twist.linear.z])
            linear_velocity = np.dot(self.transBaseCamVelocityRot, linear_velocity)    
            angular_velocity = np.array([twistMsg.twist.twist.angular.x, twistMsg.twist.twist.angular.y, twistMsg.twist.twist.angular.z])
            angular_velocity = np.dot(self.transBaseCamVelocityRot, angular_velocity)    

            current_time = (self.timeStamp - self.startTime).to_sec()

            self.riccati_obj.update_measurement(angular_velocity, linear_velocity, self.landmarks, current_time)
            solt, dt, soly, errMsg = self.riccati_obj.solver() # qua_hat_flat, p_hat_flat, input_P_flat = np.split(y, [4, 7])
            self.t = solt
            self.dt = dt
            self.estpose = soly[4:7]
            self.estori = soly[0:4]
            self.estori /= np.linalg.norm(self.estori)
            if self.DEBUG:
                print("self.estpose", self.estpose, "\nself.estori", self.estori)
            self.sendTransPose() # TODO: do we need to send the timestamp to visualization, (not sure if the timestamp will be updated)
        else:
            # self.publish_RiccatiMsg()
            pass

############################################# CALLBACKS ##############################################

############################################# SERVERS ##############################################
    # def handleGetPosition(self, req):
        # if req.trigger:
            # return GetPositionResponse(self.estpose[0],self.estpose[1], self.estpose[2])

############################################# SERVERS ##############################################
                
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    riccati_estimation_occlusion().run()