#!/usr/bin/env python3
import sys

import rospy
from robotis_controller_msgs.msg import SyncWriteItem
from std_msgs.msg import String

from python_op3.framework.modules.adaptive_walking import AdaptiveWalk


class RampWalking:
    def __init__(self, ns="/robotis"):
        self._sub_button = rospy.Subscriber(ns + "/open_cr/button", String, self._cb_button, queue_size=10)
        self._pub_sync_write = rospy.Publisher(ns + "/sync_write_item", SyncWriteItem, queue_size=0)

        self.op3_walker = AdaptiveWalk()
        self.mode = "up"
        self.set_led(1)

    def set_led(self, led):
        """
        :param led: 1, 2, 4, 7(ALL)
        :return:
        """
        msg = SyncWriteItem()
        msg.item_name = "LED"
        msg.joint_name = ["open-cr"]
        msg.value = [led]

        self._pub_sync_write.publish(msg)

    def run(self, fparam, epi):
        self.op3_walker.run("data/%s_params.json" % fparam, epi)

    def _cb_button(self, msg):
        bt = msg.data
        if bt == "mode":
            self.run(self.mode, 0)
        elif bt == "start":
            self.run(self.mode, 3)
        elif bt == "user":
            self.run(self.mode, 6)
        elif bt == "mode_long":
            if self.mode == "up":
                self.mode = "down"
                self.set_led(2)
            else:
                self.mode = "up"
                self.set_led(1)
            print("Walk mode: %s" % self.mode)
        elif bt == "start_long":
            self.set_led(4)
            if self.mode == "down":
                self.run("org", 0)
            else:
                self.run("org", 1)
        elif bt == "user_long":
            # robotis default initial pose
            pass


if __name__ == '__main__':
    _ = RampWalking()
    rospy.spin()
