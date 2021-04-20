from op3_action_module_msgs.srv import IsRunning
import rospy
from std_msgs.msg import String, Int32


class Action(object):
    def __init__(self, ns):
        self.is_action_done = True

        self._pub_ini_pose = rospy.Publisher(ns + "/base/ini_pose", String, queue_size=0)
        self._pub_action = rospy.Publisher(ns + "/action/page_num", Int32, queue_size=0)

        rospy.wait_for_service(ns + '/action/is_running')
        self.action_is_running_srv = rospy.ServiceProxy(ns + '/action/is_running', IsRunning)

    def play_motion(self, action, start_voice="好哦", done_voice=None, is_blocking=True):
        """
        Available page_num:
         -2 : BREAK
         -1 : STOP
         "ini_pose": initial pose
        #  0 : .
          4 : Thank you
          41: Introduction
          24: Wow
          23: Yes go
          15: Sit down
          1: Stand up
          54: Clap please
          27: Oops
          38: Bye bye
          111 : Intro01
          115 : Intro02
          118 : Intro03

        # soccer
          80 : Init pose
          122 : Get up(Front)
          123 : Get up(Back)
          121 : Right Kick
          120 : Left Kick
          60 : Keeper Ready
          61 : Defence to Right
          62 : Defence to Left
        # action demo
          200 : Foot play
          202 : Roll back
          204 : Look
          126 : Push up
        """
        if not self.is_action_done:
            print("still acting.")
            return
        self.is_action_done = False

        if start_voice:
            self.google_tts(start_voice)

        if action == "ini_pose":
            self._pub_ini_pose.publish(action)
            self.present_module = action
        elif isinstance(action, int) or action.isnumeric():
            action = int(action)
            self.check_module("action_module")
            self._pub_action.publish(action)

        if is_blocking:
            while not self.is_action_done:
                rospy.sleep(0.1)
            if done_voice:
                self.google_tts(done_voice)

    def go_init_pose(self, start_voice="進入賢者模式。"):
        if self.present_module != "ini_pose":
            self.torque_on()
            rospy.sleep(0.1)
            self.play_motion("ini_pose", start_voice=start_voice)

    def stand_up(self):
        self.play_motion(1, start_voice="我站起來50公分。")
