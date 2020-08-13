import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String


class VisionTrack:
    def __init__(self, ns):
        self._pub_face_posintion = rospy.Publisher("/face_position", Point)
        self._pub_demo_mode = rospy.Publisher(ns + "/mode_command", String)


