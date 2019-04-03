# -*- coding: utf-8 -*-
import rospy
from robotis_controller_msgs.msg import StatusMsg, SyncWriteItem, JointCtrlModule
from robotis_controller_msgs.srv import GetJointModule, SetJointModule
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def constructor(self):
    ns = self.ns

    self.is_module_applied = False
    self.present_module = None
    self.module_each_joint = {}
    self.angle_each_joint = {}

    self._sub_joints = rospy.Subscriber(ns + "/present_joint_states", JointState, _cb_joints, self, queue_size=10)
    self._sub_modules = rospy.Subscriber(ns + "/present_joint_ctrl_modules", JointCtrlModule, _cb_module, self,
                                         queue_size=10)
    # robotis status
    self._sub_status = rospy.Subscriber(ns + "/status", StatusMsg, _cb_status, self, queue_size=10)

    # set angle using robotis framwork
    self._pub_legacy = rospy.Publisher(ns + "/set_joint_states", JointState, queue_size=0)

    self._pub_sync_write = rospy.Publisher(ns + "/sync_write_item", SyncWriteItem, queue_size=0)
    self._pub_module = rospy.Publisher(ns + "/enable_ctrl_module", String, queue_size=0)
    self._pub_dxl_torque = rospy.Publisher(ns + "/dxl_torque", String, queue_size=0)

    # Custom function thread
    self._pub_suspend = rospy.Publisher("~/suspend", Bool, queue_size=0)

    # Services (NOT test yet!!!)
    rospy.wait_for_service(ns + '/get_present_joint_ctrl_modules')
    self.get_joint_module_srv = rospy.ServiceProxy(ns + '/get_present_joint_ctrl_modules', GetJointModule)
    rospy.wait_for_service(ns + '/set_present_joint_ctrl_modules')
    self.set_joint_module_srv = rospy.ServiceProxy(ns + '/set_present_joint_ctrl_modules', SetJointModule)

    rospy.loginfo("Waiting for joints to be populated...")
    while not rospy.is_shutdown():
        if self.angle_each_joint != {}: break
        rospy.sleep(0.1)
        rospy.loginfo("Waiting for joints to be populated...")
    rospy.loginfo("Joints populated")

    rospy.loginfo("Creating joint command publishers")


def _cb_joints(msg, self):
    self.angle_each_joint = dict(zip(msg.name, msg.position))
    # print self.angle_each_joint


def _cb_module(msg, self):
    self.module_each_joint = dict(zip(msg.joint_name, msg.module_name))
    self.is_module_applied = True


def _cb_status(msg, self):
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
        if float(msg.status_msg[15:-1]) < 11.1:
            self._pub_suspend.publish(True)
        self.google_tts("電壓剩餘：" + msg.status_msg[15:-1] + "伏特。")


def get_angles(self, joints):
    return [self.angle_each_joint[joint] for joint in joints]


def get_joint_module(self, joint_names):
    try:
        srv = GetJointModule()
        srv.joint_name = joint_names
        return self.get_joint_module_srv(srv).module_name
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_joint_module(self, joint_names, module_names):
    try:
        srv = SetJointModule()
        srv.joint_name = joint_names
        srv.module_name = module_names
        return self.set_joint_module_srv(srv)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_module(self, module, voice=None, is_blocking=True):
    if self.present_module != module:
        self._pub_module.publish(module)
        if is_blocking:
            self.is_module_applied = False
            while not self.is_module_applied:
                rospy.sleep(0.1)
        self.present_module = module
        print "Set " + module + " done."
        if voice is None:
            if self.present_module == "direct_control_module":
                self.google_tts("進入手動模式。")
            elif self.present_module == "action_module":
                self.google_tts("進入動作模式。")
        else:
            self.google_tts(voice)


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


def set_angles_legacy(self, angles):
    self._pub_module.publish("none")
    msg = JointState()
    msg.name = angles.keys()
    msg.position = angles.values()
    self._pub_legacy.publish(msg)