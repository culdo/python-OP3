import rospy
from std_msgs.msg import String


class YOLOAct:
    def __init__(self, op3):
        _ = rospy.Subscriber("/yolo_action", String, self._cb_act)
        self.is_enable = False
        self.op3 = op3

    def _cb_act(self, msg):
        hand_sign = msg.data
        print("hand_sign: %s" % hand_sign)
        if hand_sign == "Start":
            self.is_enable = True

        if self.is_enable:
            if hand_sign == "1":
                self.op3.play_motion(4, start_voice="Thank you", is_blocking=False)
            elif hand_sign == "2":
                self.op3.play_motion(24, start_voice="Wow", is_blocking=False)
            elif hand_sign == "3":
                self.op3.play_motion(23, start_voice="Yes go", is_blocking=False)
