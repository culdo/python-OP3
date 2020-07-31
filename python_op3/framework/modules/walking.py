import re

import rospy
from geometry_msgs.msg import Pose
from op3_online_walking_module_msgs.msg import FootStepCommand
from op3_online_walking_module_msgs.msg import WalkingParam as OnlineWalkingParam
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float64, String, Bool


class Walking(object):
    def __init__(self, ns):

        self.is_balance_set = False
        self.is_walking_done = False
        self.walking_query = re.compile("(?<=\[END] Walking Control \()(\d+)/(\d+)")

        # ros logging subscriber for walking control
        self._sub_rosinfo = rospy.Subscriber("/rosout", Log, self._cb_rosinfo, queue_size=10)

        # Walking
        self._pub_walking_command = rospy.Publisher(ns + "/walking/command", String, queue_size=0)
        self._pub_walking_params = rospy.Publisher(ns + "/walking/set_params", WalkingParam, queue_size=0)
        self._pub_head_scan = rospy.Publisher(ns + "/head_control/scan_command", String, queue_size=0)

        # Online Walking
        self._sub_movement_done = rospy.Subscriber(ns + "/movement_done", String, self._cb_movement_done,
                                                   queue_size=10)

        self._pub_online_walking = rospy.Publisher(ns + "/online_walking/foot_step_command", FootStepCommand,
                                                   queue_size=0)
        self._pub_online_param = rospy.Publisher(ns + "/online_walking/walking_param", OnlineWalkingParam,
                                                 queue_size=0)
        self._pub_body_offset = rospy.Publisher(ns + "/online_walking/body_offset", Pose,
                                                queue_size=0)
        self._pub_foot_distance = rospy.Publisher(ns + "/online_walking/foot_distance", Float64, queue_size=0)
        self._pub_wholebody_balance = rospy.Publisher(ns + "/online_walking/wholebody_balance_msg", String,
                                                      queue_size=0)
        self._pub_reset_body = rospy.Publisher(ns + "/online_walking/reset_body", Bool, queue_size=0)

        # Walking Services
        rospy.wait_for_service(ns + '/walking/get_params')
        self.get_walking_param_srv_ = rospy.ServiceProxy(ns + '/walking/get_params', GetWalkingParam)

    def _cb_movement_done(self, msg):
        print("movement_done: " + msg.data)
        if msg.data == "xxx":
            self.is_walking_done = True

    def _cb_rosinfo(self, msg):
        # print msg
        print("rosinfo: " + msg.msg)
        if msg.name == "/op3_manager":
            walking_state = self.walking_query.search(msg.msg)
            if walking_state:
                print("present_step: " + walking_state.group(1), "total_steps: " + walking_state.group(2))
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

    def set_body_offset(self, offset_x=0.03, offset_y=0.0, offset_z=0.0, foot_distance=0.09):
        pose = Pose()
        pose.position.x = offset_x
        pose.position.y = offset_y
        pose.position.z = offset_z
        self._pub_foot_distance.publish(foot_distance)
        self._pub_body_offset.publish(pose)
        rospy.sleep(1)

    def set_online_param(self, dsp_ratio=0.20, lipm_height=0.12, foot_height_max=0.05, zmp_offset_x=0.0,
                         zmp_offset_y=0.0):
        param = OnlineWalkingParam(dsp_ratio=dsp_ratio, lipm_height=lipm_height, foot_height_max=foot_height_max,
                                   zmp_offset_x=zmp_offset_x, zmp_offset_y=zmp_offset_y)
        self._pub_online_param.publish(param)

    def ready_for_walking(self, **kwargs):
        self.play_motion(80, start_voice=None)
        self.check_module("online_walking_module")
        self._pub_reset_body.publish(True)
        # Spend 1 secs for getting ready pose.
        rospy.sleep(1)
        self.set_balance("on")
        self.set_online_param(**kwargs)
        self.set_body_offset(**kwargs)

    def online_walking_command(self, direction="forward", step_num=4,
                               step_time=0.50, step_length=0.04, side_length=0.03,
                               step_angle=0.2, start_leg="right_leg", is_blocking=True, **kwargs):
        """
        Available direction: turn_left, turn_right, forward,
                             backward, stop, left, right
        """
        if self.present_module != "online_walking_module":
            self.ready_for_walking(**kwargs)

        self.is_walking_done = False
        msg = FootStepCommand()
        if direction in ["forward", "backward"]:
            msg.step_length = step_length
        elif direction in ["side_left", "side_right"]:
            start_leg = direction[5:] + "_leg"
            msg.side_length = side_length
        elif direction in ["turn_left", "turn_right"]:
            start_leg = direction[5:] + "_leg"
            msg.step_angle = step_angle
        else:
            print("Dangerous!!! 停止步行")
            return
        msg.command = direction
        msg.step_time = step_time
        msg.step_num = step_num
        msg.start_leg = start_leg
        self._pub_online_walking.publish(msg)
        if is_blocking:
            while not self.is_walking_done:
                rospy.sleep(0.1)
            rospy.sleep(1)
