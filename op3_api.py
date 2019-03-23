# -*- coding: utf-8 -*-
import os
import subprocess
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String, Float64, Bool
from robotis_controller_msgs.msg import StatusMsg, SyncWriteItem, JointCtrlModule
from robotis_controller_msgs.srv import GetJointModule, SetJointModule
from op3_online_walking_module_msgs.msg import FootStepCommand
from op3_walking_module_msgs.srv import GetWalkingParam, SetWalkingParam
from op3_walking_module_msgs.msg import WalkingParam
from op3_action_module_msgs.srv import IsRunning
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Log
# For pydev debugger
# import sys
# reload(sys)
# sys.setdefaultencoding('utf-8')
import requests
import re


class Op3(object):
    """
    Client ROS class for manipulating Robotis-OP3 in Gazebo
    """

    def __init__(self, ns="/robotis", model_name="robotis_op3"):
        rospy.init_node("op3_tester")
        self.ns = ns
        self.model_name = model_name
        self.link_state = None
        self.is_action_done = False
        self.is_module_applied = False
        self.present_module = None
        self.is_balance_set = False
        self.is_walking_done = False
        self.module_each_joint = {}
        self.angle_each_joint = {}
        self.walking_query = re.compile("(?<=Walking Control \()(\d+)/(\d+)")

        ## Subscribers
        # get angle and module on each joint
        self._sub_joints = rospy.Subscriber(ns + "/present_joint_states", JointState, self._cb_joints, queue_size=10)
        self._sub_modules = rospy.Subscriber(ns + "/present_joint_ctrl_modules", JointCtrlModule, self._cb_module,
                                             queue_size=10)
        # robotis status
        self._sub_status = rospy.Subscriber(ns + "/status", StatusMsg, self._cb_status, queue_size=10)

        # open_cr_module
        self._sub_imu = rospy.Subscriber(ns + "/open_cr/imu", Imu)
        self._sub_button = rospy.Subscriber(ns + "/open_cr/button", String)
        # ros logging subscriber for walking control
        self._sub_rosinfo = rospy.Subscriber("/rosout", Log, self._cb_rosinfo, queue_size=10)

        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.angle_each_joint != {}: break
            rospy.sleep(0.1)
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")

        rospy.loginfo("Creating joint command publishers")

        ## Publishers
        # Controller
        self._pub_sync_write = rospy.Publisher(ns + "/sync_write_item", SyncWriteItem, queue_size=0)
        self._pub_module = rospy.Publisher(ns + "/enable_ctrl_module", String, queue_size=0)
        self._pub_dxl_torque = rospy.Publisher(ns + "/dxl_torque", String, queue_size=0)

        # Action
        self._pub_ini_pose = rospy.Publisher(ns + "/base/ini_pose", String, queue_size=0)
        self._pub_action = rospy.Publisher(ns + "/action/page_num", Int32, queue_size=0)

        # set angle using direct_control_module
        self._pub_joints = rospy.Publisher(ns + "/direct_control/set_joint_states", JointState, queue_size=0)
        # set angle using robotis framwork
        self._pub_legacy = rospy.Publisher(ns + "/set_joint_states", JointState, queue_size=0)

        # Walking
        self._pub_walking = rospy.Publisher(ns + "/walking/command", String, queue_size=0)
        self._pub_walking_params = rospy.Publisher(ns + "/walking/set_params", WalkingParam, queue_size=0)
        self._pub_head_scan = rospy.Publisher(ns + "/head_control/scan_command", String, queue_size=0)

        # Online Walking
        self._pub_online_walking = rospy.Publisher(ns + "/online_walking/foot_step_command", FootStepCommand,
                                                   queue_size=0)
        self._pub_foot_distance = rospy.Publisher(ns + "/online_walking/foot_distance", Float64, queue_size=0)
        self._pub_wholebody_balance = rospy.Publisher(ns + "/online_walking/wholebody_balance_msg", String,
                                                      queue_size=0)
        self._pub_reset_body = rospy.Publisher(ns + "/online_walking/reset_body", Bool, queue_size=0)

        # Utility
        self._pub_sound = rospy.Publisher("/play_sound_file", String, queue_size=0)

        # Services (NOT test yet!!!)
        rospy.wait_for_service(ns + '/get_present_joint_ctrl_modules')
        self.get_joint_module = rospy.ServiceProxy(ns + '/get_present_joint_ctrl_modules', GetJointModule)
        rospy.wait_for_service(ns + '/set_present_joint_ctrl_modules')
        self.set_joint_module = rospy.ServiceProxy(ns + '/set_present_joint_ctrl_modules', SetJointModule)
        rospy.wait_for_service(ns + '/action/is_running')
        self.action_is_running = rospy.ServiceProxy(ns + '/action/is_running', IsRunning)

        rospy.sleep(0.5)
        self.go_init_pose()

    def google_tts(self, query="%E6%88%91%E6%98%AFTTSAPI", lang="zh"):
        doc = requests.get(
            "https://translate.google.com/translate_tts?ie=UTF-8&q=" + query + "&tl=" + lang + "&client=tw-ob")
        with open('buff.mp3', 'wb') as f:
            f.write(doc.content)
        self._pub_sound.publish(os.path.join(os.getcwd(), "buff.mp3"))

    def set_default_moving_time(self, param):
        rospy.set_param(self.ns + "/direct_control/default_moving_time", param)

    def set_default_moving_angle(self, param):
        rospy.set_param(self.ns + "/direct_control/default_moving_angle", param)

    def set_check_collision(self, param):
        rospy.set_param(self.ns + "/direct_control/check_collision", param)

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
                               step_angle=0.1, start_leg="right_leg", is_blocking=True):
        """
        Available direction: turn_left, turn_right, forward,
                             backward, stop, left, right
        """
        msg = FootStepCommand()
        msg.command = direction
        msg.start_leg = start_leg
        msg.step_time = step_time
        msg.step_num = step_num
        if direction == "forward" or direction == "backward":
            msg.step_length = step_length
        elif direction == "left" or direction == "right":
            msg.side_length = side_length
        elif direction == "turn_left" or direction == "turn_right":
            msg.step_angle = step_angle
        else:
            print "停止步行"
        self._pub_online_walking.publish(msg)
        if is_blocking:
            self.is_walking_done = False
            while not self.is_walking_done:
                rospy.sleep(0.1)

    def safety_suspend(self):
        self.go_init_pose()
        self.set_module("direct_control_module")
        self.set_angles({"l_el": 0,
                         "r_el": 0})
        rospy.sleep(2)
        self.torque_off()

    def head_control(self, pan, tilt):
        self.set_angles({"head_pan": np.pi * (pan / 180.0),
                         "head_tilt": np.pi * (tilt / 180.0)})

    def set_walk_velocity(self, x, y, t):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        self._pub_cmd_vel.publish(msg)

    def stand_up(self):
        self.play_motion(1, voice="我站起來50公分。")

    def set_balance(self, sw):
        self.is_balance_set = False
        self._pub_wholebody_balance.publish("balance_" + sw)
        while not self.is_balance_set:
            rospy.sleep(0.1)

    def set_module(self, module, is_blocking=True):
        if self.present_module != module:
            self._pub_module.publish(module)
            if is_blocking:
                self.is_module_applied = False
                while not self.is_module_applied:
                    rospy.sleep(0.1)
            self.present_module = module
            print "Set " + module + " done."
            if self.present_module == "direct_control_module":
                self.google_tts("進入手動模式。")
            elif self.present_module == "action_module":
                self.google_tts("進入動作模式。")

    def torque_off(self, joint_names=None):
        msg = SyncWriteItem()
        msg.item_name = "torque_enable"
        if joint_names is not None:
            msg.joint_name = joint_names
        else:
            msg.joint_name = self.angle_each_joint.keys()
        msg.value = [0 for _ in range(len(msg.joint_name))]
        self._pub_sync_write.publish(msg)

    def torque_on(self, joint_names=None):
        if joint_names is not None:
            # Warning: This condition is not safety.
            self.set_angles_legacy(self.angle_each_joint)
            msg = SyncWriteItem()
            msg.item_name = "torque_enable"
            msg.joint_name = joint_names
            msg.value = [1 for _ in range(len(msg.joint_name))]
            self._pub_sync_write.publish(msg)
        else:
            self._pub_dxl_torque.publish("check")

    def go_init_pose(self):
        self.torque_on()
        rospy.sleep(0.5)
        self.play_motion("ini_pose", voice="糙你媽我腿要斷了。")

    def _cb_rosinfo(self, msg):
        # print msg
        if msg.name == "/op3_manager":
            print msg.msg
            walking_state = self.walking_query.search(msg.msg)
            if walking_state:
                print "present_step: " + walking_state.group(1), "total_steps: " + walking_state.group(2)
                if walking_state.group(1) == walking_state.group(2):
                    self.is_walking_done = True
            elif msg.msg == "[END] Balance Gain":
                self.is_balance_set = True

    def _cb_joints(self, msg):
        self.angle_each_joint = dict(zip(msg.name, msg.position))
        # print self.angle_each_joint

    def _cb_module(self, msg):
        self.module_each_joint = dict(zip(msg.joint_name, msg.module_name))
        self.is_module_applied = True

    def _cb_status(self, msg):
        if msg.module_name == "Action" or msg.module_name == "Base":
            print msg.status_msg
            if msg.status_msg == "Action_Finish" or \
                    msg.status_msg == "Finish Init Pose" or \
                    msg.status_msg[:6] == "Failed":
                self.is_action_done = True
                if msg.status_msg == "Finish Init Pose":
                    self.is_module_applied = False
            # else:
            #     self.google_tts("動作開始")
        elif msg.module_name == "SENSOR":
            print msg.status_msg
            self.google_tts("電壓剩餘：" + msg.status_msg[15:-1] + "伏特。")

    def play_motion(self, action, voice="動作完成。", is_blocking=True):
        """
        Available page_num:
         -2 : BREAK
         -1 : STOP
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
        if action == "ini_pose":
            self._pub_ini_pose.publish(action)
            self.present_module = action
        else:
            self.set_module("action_module")
            self._pub_action.publish(action)

        if is_blocking:
            self.is_action_done = False
            while not self.is_action_done:
                rospy.sleep(0.1)
        self.google_tts(voice)

    def get_angles(self, joints):
        return [self.angle_each_joint[joint] for joint in joints]

    def set_angles(self, angles):
        # self.set_module("direct_control_module")

        msg = JointState()
        msg.name = angles.keys()
        msg.position = angles.values()
        self._pub_joints.publish(msg)

    def set_angles_legacy(self, angles):
        self._pub_module.publish("none")
        msg = JointState()
        msg.name = angles.keys()
        msg.position = angles.values()
        self._pub_legacy.publish(msg)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if t > stop: break
            ratio = (t - start) / delay
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            r.sleep()

    def wave_angle(self, test_joints, angles=None, duration=2, print_angle=False):
        if angles is None:
            angles = np.random.uniform(1.57, -1.57, 3)
        angle_dict = dict(zip(test_joints, angles))
        self.set_angles(angle_dict)
        if print_angle:
            rospy.loginfo(np.round(angle_dict.values(), 2))
        rospy.sleep(duration=duration)

    def get_joint_module(self, joint_names):
        try:
            srv = GetJointModule()
            srv.joint_name = joint_names
            return self.get_joint_module(srv).module_name
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_joint_module(self, joint_names, module_names):
        try:
            srv = SetJointModule()
            srv.joint_name = joint_names
            srv.module_name = module_names
            return self.set_joint_module(srv)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0: return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


def arm_robot():
    run_move_arm = "roslaunch mstp_op3_move_arm op3_move_arm.launch".split()
    try:
        master = subprocess.Popen(run_move_arm)
    except:
        master.terminate()

    while True:
        try:
            rospy.get_published_topics()
            break
        except:
            rospy.sleep(0.1)

    # atexit.register(clear_exit, master)


def clear_exit(p, gz=None):
    p.terminate()
    if gz is not None:
        gz.terminate()


if "__main__" == __name__:
    op3_tester = Op3()
    # op3_tester.ready_for_walking()
    # op3_tester.safety_suspend()
    pass
