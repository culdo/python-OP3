import rospy
from framework import Framework


class Op3(Framework):
    """
    Client ROS class for manipulating Robotis-OP3 in real-world and Gazebo
    """
    def __init__(self, ns="/robotis", is_init=True):
        rospy.init_node("op3_tester")
        # exported method
        super(Op3, self).__init__(ns)

        rospy.sleep(0.5)
        if is_init:
            self.go_init_pose()
