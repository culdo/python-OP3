# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool


def constructor(self):
    self._sub_suspend = rospy.Subscriber("~/suspend", Bool, _cb_suspend, self, queue_size=10)


def safety_suspend(self):
    self.go_init_pose()
    self.set_module("direct_control_module", voice="林北要軟了，我說身體。")
    self.set_angles({"l_el": 0,
                     "r_el": 0})
    rospy.sleep(2)
    self.torque_off()


def _cb_suspend(msg, self):
    if msg.data is True:
        if self.present_module == "action_module":
            self.play_motion(-2)
        elif self.present_module == "online_walking_module":
            self.online_walking_command(direction="stop")
        self.safety_suspend()