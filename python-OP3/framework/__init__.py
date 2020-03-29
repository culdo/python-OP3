import rospy
from std_msgs.msg import Bool
from .modules.robotis_controller import Controller
from .modules.op3_utility import Utility
from .modules.op3_action import Action
from .modules.op3_direct_control import DirectControl
from .modules.op3_walking import Walking
from .modules.open_cr import OpenCR
from .modules.usb_cam import UsbCam


class Framework(Controller, Utility, Action, DirectControl,
                Walking, OpenCR, UsbCam):
    def __init__(self, ns):
        self.ns = ns
        Controller.__init__(self)
        Utility.__init__(self)
        Action.__init__(self)
        DirectControl.__init__(self)
        Walking.__init__(self)
        OpenCR.__init__(self)
        UsbCam.__init__(self)

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
                self.play_motion(-2, "林北站起來了")
            elif self.present_module == "online_walking_module":
                self.online_walking_command(direction="stop")
        self.safety_suspend()

