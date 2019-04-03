# -*- coding: utf-8 -*-
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import rospy


def constructor(self):
    ns = self.ns
    self._sub_imu = rospy.Subscriber(ns + "/open_cr/imu", Imu, _cb_imu, self, queue_size=10)
    self._sub_button = rospy.Subscriber(ns + "/open_cr/button", String, _cb_button, self, queue_size=10)


def _cb_imu(msg, self):
    # print msg
    pass


def _cb_button(msg, self):
    # print msg
    pass
