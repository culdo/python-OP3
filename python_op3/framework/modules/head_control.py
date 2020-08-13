import numpy as np
import rospy
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HeadControl(object):
    def __init__(self, ns):
        self.ns = ns
        _ = rospy.Subscriber("/rosout", Log, self._cb_headinfo, queue_size=10)

        self._pub_head_scan = rospy.Publisher(ns + "/head_control/scan_command", String)
        self._pub_head_joints = rospy.Publisher(ns + "/head_control/set_joint_states", JointState)
        self._pub_head_offset = rospy.Publisher(ns + "/head_control/set_joint_states_offset", JointState)
        self.is_head_done = False

    def _cb_headinfo(self, msg):
        if msg.name == "/op3_demo_opc":
            if msg.msg == "Head movement is finished.":
                self.is_head_done = True

    def set_head(self, pan, tilt, offset=False, blocking=False):
        self.is_head_done = False
        self.check_module("head_control_module")
        msg = JointState()
        msg.name = ["head_pan", "head_tilt"]
        msg.position = [np.pi * (pan / 180.0),
                        np.pi * (tilt / 180.0)]
        if offset:
            self._pub_head_offset.publish(msg)
        else:
            self._pub_head_joints.publish(msg)

        if blocking:
            while not self.is_head_done:
                rospy.sleep(0.01)

    def stop_head_scan(self):
        self._pub_head_scan.publish("stop")

    def head_scan(self):
        self._pub_head_scan.publish("scan")
