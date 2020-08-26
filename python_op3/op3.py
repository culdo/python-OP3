import rospy
from python_op3.voice_comm.voice_control import VoiceController
from std_msgs.msg import Bool

from .framework.core.open_cr import OpenCR
from .framework.core.robotis_controller import Controller
from .framework.core.usb_cam import UsbCam
from .framework.modules.action import Action
from .framework.modules.direct_control import DirectControl
from .framework.modules.head_control import HeadControl
from .framework.modules.utility import Utility
from .framework.modules.online_walking import OnlineWalking
from .framework.modules.walking import Walk


class Op3(Controller, Utility, Action, DirectControl, HeadControl,
          OnlineWalking, OpenCR, UsbCam):
    """
    Client ROS class for manipulating Robotis-OP3 in real-world and Gazebo
    """

    def __init__(self, ns="/robotis", is_init=True, vc_forever=False):
        rospy.init_node("op3_tester")

        Controller.__init__(self, ns)
        Utility.__init__(self)
        Action.__init__(self, ns)
        DirectControl.__init__(self, ns)
        HeadControl.__init__(self, ns)
        OnlineWalking.__init__(self, ns)
        OpenCR.__init__(self, ns)
        UsbCam.__init__(self)
        self.vc = VoiceController(self, vc_forever)
        self.walker = Walk()
        # _ = YOLOAct(self)

        self._sub_suspend = rospy.Subscriber("~/suspend", Bool, self._cb_suspend, queue_size=10)

        rospy.sleep(0.5)
        if is_init:
            self.go_init_pose()

    def _safety_suspend(self):
        self.go_init_pose()
        self.check_module("direct_control_module", voice="林北要軟了，我說身體。")
        self.set_angles({"l_el": 0,
                         "r_el": 0})
        rospy.sleep(2)
        self.torque_off()

    def _cb_suspend(self, msg):
        if msg.data is True:
            if self.present_module == "action_module":
                self.play_motion(-2, "林北站起來了")
            elif self.present_module == "online_walking_module":
                self.online_walking_command(direction="stop")
        self._safety_suspend()
