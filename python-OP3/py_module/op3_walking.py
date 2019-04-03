# -*- coding: utf-8 -*-
import rospy
from op3_walking_module_msgs.srv import GetWalkingParam, SetWalkingParam
from op3_walking_module_msgs.msg import WalkingParam
from op3_online_walking_module_msgs.msg import FootStepCommand
from std_msgs.msg import Float64, String, Bool
from rosgraph_msgs.msg import Log
import re


class Walking(object):
    def __init__(self):

        self.is_balance_set = False
        self.is_walking_done = False
        self.walking_query = re.compile("(?<=Walking Control \()(\d+)/(\d+)")

        # ros logging subscriber for walking control
        self._sub_rosinfo = rospy.Subscriber("/rosout", Log, self._cb_rosinfo, queue_size=10)

        # Walking
        self._pub_walking = rospy.Publisher(self.ns + "/walking/command", String, queue_size=0)
        self._pub_walking_params = rospy.Publisher(self.ns + "/walking/set_params", WalkingParam, queue_size=0)
        self._pub_head_scan = rospy.Publisher(self.ns + "/head_control/scan_command", String, queue_size=0)

        # Online Walking
        self._sub_movement_done = rospy.Subscriber(self.ns + "/movement_done", String, self._cb_movement_done,
                                                   queue_size=10)

        self._pub_online_walking = rospy.Publisher(self.ns + "/online_walking/foot_step_command", FootStepCommand,
                                                   queue_size=0)
        self._pub_foot_distance = rospy.Publisher(self.ns + "/online_walking/foot_distance", Float64, queue_size=0)
        self._pub_wholebody_balance = rospy.Publisher(self.ns + "/online_walking/wholebody_balance_msg", String,
                                                      queue_size=0)
        self._pub_reset_body = rospy.Publisher(self.ns + "/online_walking/reset_body", Bool, queue_size=0)

    def _cb_movement_done(self, msg):
        print "movement_done: " + msg.data
        if msg.data == "xxx":
            self.is_walking_done = True

    def _cb_rosinfo(self, msg):
        # print msg
        if msg.name == "/op3_manager":
            print "rosinfo: " + msg.msg
            walking_state = self.walking_query.search(msg.msg)
            if walking_state:
                print "present_step: " + walking_state.group(1), "total_steps: " + walking_state.group(2)
                if walking_state.group(1) == walking_state.group(2):
                    self.is_walking_done = True
            elif msg.msg == "[END] Balance Gain":
                self.is_balance_set = True
            elif msg.msg == "[END] Joint Control":
                self.present_module = "online_walking_module"
            elif msg.msg == "Walking Enable":
                self.present_module = "op3_walking_module"

    def set_balance(self, sw):
        self.is_balance_set = False
        self._pub_wholebody_balance.publish("balance_" + sw)
        while not self.is_balance_set:
            rospy.sleep(0.1)

    def ready_for_walking(self):
        self.set_module("online_walking_module")
        self._pub_reset_body.publish(True)
        # Getting ready pose spending 5 secs.
        rospy.sleep(5)
        self.set_balance("on")
        self._pub_foot_distance.publish(0.09)
        rospy.sleep(0.5)

    def online_walking_command(self, direction="forward", step_num=4,
                               step_time=0.50, step_length=0.04, side_length=0.03,
                               step_angle=0.1, start_leg=None, is_blocking=True):
        """
        Available direction: turn_left, turn_right, forward,
                             backward, stop, left, right
        """
        if self.present_module != "online_walking_module":
            self.ready_for_walking()
        msg = FootStepCommand()
        msg.command = direction
        msg.step_time = step_time
        msg.step_num = step_num
        if direction == "forward" or direction == "backward":
            start_leg = "right_leg"
            msg.step_length = step_length
        elif direction == "left" or direction == "right":
            msg.side_length = side_length
        elif direction == "turn_left" or direction == "turn_right":
            msg.step_angle = step_angle
        else:
            print "停止步行"
        msg.start_leg = start_leg
        self._pub_online_walking.publish(msg)
        if is_blocking:
            self.is_walking_done = False
            while not self.is_walking_done:
                rospy.sleep(0.1)
