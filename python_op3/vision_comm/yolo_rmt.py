import rospy
from std_msgs.msg import String


class YOLOAct:
    def __init__(self, op3):
        _ = rospy.Subscriber("/yolo_action", String, self._cb_act)
        self.is_enable = False
        self.op3 = op3

    def _cb_act(self, msg):
        hand_sign = msg.data
        if hand_sign == "Start":
            self.is_enable = True

        if self.is_enable:
            if hand_sign == "1":
                self.op3.play_motion(4, is_blocking=False)
                # self.op3.play_motion(61, is_blocking=False)
            elif hand_sign == "2":
                self.op3.play_motion(24, is_blocking=False)
                # self.op3.play_motion(200, is_blocking=False)
            elif hand_sign == "3":
                self.op3.play_motion(23, is_blocking=False)
                # self.op3.play_motion(126, is_blocking=False)
