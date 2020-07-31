import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HeadControl(object):
    def __init__(self, ns):
        self.ns = ns
        self._pub_head_scan = rospy.Publisher(ns + "/head_control/scan_command", String)
        self._pub_head_joints = rospy.Publisher(ns + "/head_control/set_joint_states", JointState)
        self._pub_head_offset = rospy.Publisher(ns + "/head_control/set_joint_states_offset", JointState)

    def set_head(self, pan, tilt, offset=False):
        self.check_module("head_control_module")
        msg = JointState()
        msg.name = ["head_pan", "head_tilt"]
        msg.position = [np.pi * (pan / 180.0),
                        np.pi * (tilt / 180.0)]
        if offset:
            self._pub_head_offset.publish(msg)
        else:
            self._pub_head_joints.publish(msg)

    def stop_head_scan(self):
        self._pub_head_scan.publish("stop")

    def head_scan(self):
        self._pub_head_scan.publish("scan")
