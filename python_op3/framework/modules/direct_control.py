# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time


class DirectControl(object):
    def __init__(self, ns):
        self.ns = ns
        self._pub_joints = rospy.Publisher(ns + "/direct_control/set_joint_states", JointState, queue_size=0)

    def set_default_moving_time(self, param):
        rospy.set_param(self.ns + "/direct_control/default_moving_time", param)

    def set_default_moving_angle(self, param):
        rospy.set_param(self.ns + "/direct_control/default_moving_angle", param)

    def set_check_collision(self, param):
        rospy.set_param(self.ns + "/direct_control/check_collision", param)

    def head_control(self, pan, tilt):
        self.set_angles({"head_pan": np.pi * (pan / 180.0),
                         "head_tilt": np.pi * (tilt / 180.0)})

    def set_angles(self, angles):
        self.check_module("direct_control_module")

        msg = JointState()
        msg.name = angles.keys()
        msg.position = angles.values()
        self._pub_joints.publish(msg)

    def wave_angle(self, test_joints, angles=None, duration=2, print_angle=False):
        if angles is None:
            angles = np.random.uniform(1.57, -1.57, 3)
        angle_dict = dict(zip(test_joints, angles))
        self.set_angles(angle_dict)
        if print_angle:
            rospy.loginfo(np.round(angle_dict.values(), 2))
        rospy.sleep(duration=duration)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if t > stop: break
            ratio = (t - start) / delay
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            r.sleep()


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z
