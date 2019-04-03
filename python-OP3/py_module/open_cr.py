# -*- coding: utf-8 -*-
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import rospy


class OpenCR(object):
    def __init__(self):
        self._sub_imu = rospy.Subscriber(self.ns + "/open_cr/imu", Imu, self._cb_imu, queue_size=10)
        self._sub_button = rospy.Subscriber(self.ns + "/open_cr/button", String, self._cb_button, queue_size=10)

    def _cb_imu(self, msg):
        # print msg
        pass

    def _cb_button(self, msg):
        # print msg
        pass
