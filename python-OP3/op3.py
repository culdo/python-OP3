# -*- coding: utf-8 -*-
from py_module import *
import rospy

# For pydev debugger
# import sys
# reload(sys)
# sys.setdefaultencoding('utf-8')


class Op3(object):
    """
    Client ROS class for manipulating Robotis-OP3 in real-world and Gazebo
    """
    # imported method
    go_init_pose = op3_action.go_init_pose
    play_motion = op3_action.play_motion

    set_module = robotis_controller.set_module
    torque_off = robotis_controller.torque_off
    torque_on = robotis_controller.torque_on

    set_angles = op3_direct_control.set_angles
    online_walking_command = op3_walking.online_walking_command
    google_tts = op3_utility.google_tts
    safety_suspend = useful_method.safety_suspend

    def __init__(self, ns="/robotis"):
        rospy.init_node("op3_tester")
        # exported method
        self.ns = ns
        op3_action.constructor(self)
        op3_walking.constructor(self)
        op3_direct_control.constructor(self)
        op3_utility.constructor(self)
        open_cr.constructor(self)
        robotis_controller.constructor(self)
        useful_method.constructor(self)

        rospy.sleep(0.5)
        self.go_init_pose()


if "__main__" == __name__:
    op3_tester = Op3()
    # op3_tester.ready_for_walking()
    # op3_tester.safety_suspend()
    pass
