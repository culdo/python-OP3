#!/usr/bin/env python3
import sys

import rospy
from robotis_controller_msgs.msg import SyncWriteItem
from std_msgs.msg import String

from python_op3.framework.modules.walking import Walk


class RampWalking:
    def __init__(self, ns="/robotis"):
        self._sub_button = rospy.Subscriber(ns + "/open_cr/button", String, self._cb_button, queue_size=10)
        self._pub_sync_write = rospy.Publisher(ns + "/sync_write_item", SyncWriteItem, queue_size=0)

        self.op3_walker = Walk()
        self.mode = "up_params.json"
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

    def _cb_button(self, msg):
        bt = msg.data
        if bt == "mode":
            self.op3_walker.run(self.mode, 0)
        elif bt == "start":
            self.op3_walker.run(self.mode, 3)
        elif bt == "user":
            self.op3_walker.run(self.mode, 6)
        elif bt == "mode_long":
            if self.mode == "up_params.json":
                self.mode = "down_params.json"
                self.set_led(2)
            else:
                self.mode = "up_params.json"
                self.set_led(1)
            print("Walk mode: %s" % self.mode)
        elif bt == "start_long":
            self.set_led(4)
            if self.mode == "down_params.json":
                self.op3_walker.run("org_params.json", 0)
            else:
                self.op3_walker.run("org_params.json", 1)
        elif bt == "user_long":
            # robotis default initial pose
            pass


if __name__ == '__main__':
    _ = RampWalking()
    rospy.spin()
