#! /usr/bin/env python3

"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""

import rospy
import numpy as np
import tf2_ros
import math
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, TransformStamped, Point, Quaternion, Vector3, Vector3Stamped, PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from svea_msgs.msg import lli_ctrl
from svea.interfaces import ActuationInterface

class PurePursuitController():

    k = 0.6  # look forward gain
    Lfc = 0.2  # look-ahead distance
    K_p = 1.0  # speed control propotional gain
    K_i = 0.2  # speed control integral gain
    L = 0.324  # [m] wheel base of vehicle

    def __init__(self):
        rospy.init_node('leader_vehicle')
        self.NAMESPACE = rospy.get_namespace()
        self.SVEANAME = rospy.get_param("~svea_name")
        self.DEBUG = rospy.get_param("~debug", False)

        self.dt = rospy.get_param("~pure_pursuit_dt", 0.01)

        traj = rospy.get_param("~traj")
        traj = np.array(traj).T
        self.traj_x = traj[0]
        self.traj_y = traj[1] #rospy.get_param("~pure_pursuit_dt", 0.01)
        self.target = None

        # initialize with 0 velocity
        self.target_velocity = 0.26
        self.error_sum = 0.0
        self.last_index = 0
        self.is_finished = False
        self.computing_control = False

        self.pose = np.array([None, None, None]) #x, y, yaw
        self.vel = None # linvelx 
        self.act = ActuationInterface()
        self.act.start()
        self.delay = 20
        for i in range(self.delay):
            if not rospy.is_shutdown():
                rospy.sleep(1)
                rospy.loginfo(f"delay...{i}/{self.delay}")
            else:
                break
        rospy.Subscriber("/qualisys/" + self.SVEANAME + "/pose", PoseStamped, self.poseCallback)
        rospy.Subscriber("/agent8/actuation_twist", TwistWithCovarianceStamped, self.velCallback)

    def poseCallback(self, msg):
        euler = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, euler[2]], dtype=np.float64)
        if self.DEBUG:
            print(self.pose)

    def velCallback(self, msg):
        self.vel = msg.twist.twist.linear.x
        if self.pose.all() != None and not self.computing_control:
            self.computing_control = True
            self.compute_control()
            self.computing_control = False

    def compute_control(self, target=None):
        steering = self.compute_steering(target)
        velocity = self.compute_velocity()
        if self.DEBUG:
            print(f"steering: {steering}, velocity, {velocity}")
        self.act.send_control(steering, velocity)
        return steering, velocity

    def compute_steering(self, target=None):
        if self.is_finished:
            return 0.0
        if target is None:
            self.find_target()
        else:
            # allow manual setting of target
            self.target = target

        tx, ty = self.target
        alpha = math.atan2(ty - self.pose[1], tx - self.pose[0]) - self.pose[2]
        if self.vel < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * self.vel + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self):
        if self.is_finished:
            return 0.0
        
        # speed control
        error = self.target_velocity - self.vel
        self.error_sum += error * self.dt
        P = error * self.K_p
        I = self.error_sum * self.K_i
        correction = P + I
        return self.target_velocity + correction

    def find_target(self):
        ind = self._calc_target_index()
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)

    def _calc_target_index(self):
        # search nearest point index
        dx = [self.pose[0] - icx for icx in self.traj_x]
        dy = [self.pose[1] - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * self.vel + self.Lfc

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        if dist < 0.05:
            self.is_finished = True
            pass

        return ind
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    PurePursuitController().run()