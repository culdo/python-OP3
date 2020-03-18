# -*- coding: utf-8 -*-
import rospy
from py_module.framework import Framework
# For pydev debugger
# import sys
# reload(sys)
# sys.setdefaultencoding('utf-8')


class Op3(Framework):
    """
    Client ROS class for manipulating Robotis-OP3 in real-world and Gazebo
    """
    def __init__(self, ns="/robotis"):
        self.ns = ns
        rospy.init_node("op3_tester")
        # exported method
        Framework.__init__(self)

        rospy.sleep(0.5)
        self.go_init_pose()


if "__main__" == __name__:
    op3_tester = Op3()
    # op3_tester.ready_for_walking()
    op3_tester.safety_suspend()
    pass
