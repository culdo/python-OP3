# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
from robotis_controller import Controller
from op3_utility import Utility
from op3_action import Action
from op3_direct_control import DirectControl
from op3_walking import Walking
from open_cr import OpenCR


class Framework(Controller, Utility, Action, DirectControl, Walking, OpenCR):
    def __init__(self):
        Controller.__init__(self)
        Utility.__init__(self)
        Action.__init__(self)
        DirectControl.__init__(self)
        Walking.__init__(self)
        OpenCR.__init__(self)

        self._sub_suspend = rospy.Subscriber("~/suspend", Bool, self._cb_suspend, queue_size=10)

    def safety_suspend(self):
        self.go_init_pose()
        self.check_module("direct_control_module", voice="林北要軟了，我說身體。")
        self.set_angles({"l_el": 0,
                         "r_el": 0})
        rospy.sleep(2)
        self.torque_off()

    def _cb_suspend(self, msg):
        if msg.data is True:
            if self.present_module == "action_module":
                self.play_motion(-2)
            elif self.present_module == "online_walking_module":
                self.online_walking_command(direction="stop")
        self.safety_suspend()
