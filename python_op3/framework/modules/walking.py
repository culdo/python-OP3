#!/usr/bin/env python3
import json
from threading import Thread

import numpy as np
import rospy
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam
from robotis_controller_msgs.msg import StatusMsg
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# from base.step_state import StepState

walking_params = [
    "init_x_offset",  # [m]
    "init_y_offset",  # [m]
    "init_z_offset",  # [m]

    "init_roll_offset",  # [radius]
    "init_pitch_offset",  # [radius]
    "init_yaw_offset",  # [radius]

    "hip_pitch_offset",  # [radius]
    "period_time",  # [ms]
    "dsp_ratio",  # [double_stance_time/single_stance_time]

    "step_fb_ratio",  # [m]
    "z_move_amplitude",  # foot_height      [m]
    "y_swap_amplitude",  # swing_right_left [m]

    "z_swap_amplitude",  # swing_top_down   [m]
    "pelvis_offset",  # [radius]
    "arm_swing_gain"  # [m]
]


class Walk:
    def __init__(self, ns="/robotis"):
        self.step_count = 0
        rospy.init_node("test_walker")
        # Start walking
        _ = rospy.Subscriber(ns + "/status", StatusMsg, self._cb_status, queue_size=10)
        _ = rospy.Subscriber(ns + "/open_cr/imu", Imu, self._cb_imu, queue_size=10)
        # start, stop, balance on, balance off, save
        self._pub_cmd = rospy.Publisher(ns + "/walking/command", String, queue_size=10)
        self._pub_set_params = rospy.Publisher(ns + "/walking/set_params", WalkingParam, queue_size=10)
        self._pub_module = rospy.Publisher(ns + "/enable_ctrl_module", String, queue_size=0)

        # self.step_state = StepState()
        rospy.wait_for_service(ns + "/walking/get_params")
        self.get_param = rospy.ServiceProxy(ns + "/walking/get_params", GetWalkingParam)
        self.is_stop = True
        self.imu = None

        ddpg_steps = 64
        webot_time_step = 16
        time_unit = 8
        self.period_steps = ddpg_steps * (webot_time_step / time_unit)

        rospy.on_shutdown(self.stop)
        rospy.sleep(0.5)

    def _cb_status(self, msg):
        if msg.module_name == "Walking":
            self.status = msg.status_msg
            if "step time_:" in self.status:
                self.step_count += 1
                print(self.step_count)

    def _cb_imu(self, msg):
        self.imu = msg

    def _update_walking_params(self, param_list):
        param_dict = dict(zip(walking_params, param_list))
        param = self.get_param().parameters
        param.x_move_amplitude = 0.020
        for k, v in param_dict.items():
            if k in ["init_x_offset", "init_y_offset", "init_z_offset", "period_time",
                     "x_swap_amplitude", "y_swap_amplitude", "z_swap_amplitude", "z_move_amplitude"]:
                v *= 0.001
            elif k in ["init_roll_offset", "init_pitch_offset", "init_yaw_offset",
                       "hip_pitch_offset", "pelvis_offset"]:
                v *= (np.pi / 180)
            if k == "arm_swing_gain" or v < 1.0 or (k == "step_fb_ratio" and v < 1.20):
                param.__setattr__(k, v)
            else:
                raise AssertionError("Danger! Out of parameter range. -> %s:%.2f" % (k, v))
        self._pub_set_params.publish(param)

    def get_param_list(self):
        param = self.get_param()
        for p in param.__dict__:
            pass

    def stop(self):
        print("Stop walk.")
        self._pub_cmd.publish("stop")
        self.is_stop = True

    def run_th(self, json_file, ep_i=0):
        self.is_stop = False
        step_params = self._read_ep_params(json_file, ep_i)
        # Set pose
        self._update_walking_params(step_params[0])
        rospy.sleep(5)
        print("json_file:%s epi:%s" % (json_file, ep_i))
        # Enable walking module
        self._pub_module.publish("walking_module")
        self._pub_cmd.publish("start")

        if len(step_params) == 1:
            step_params.extend([[] for _ in range(14)])
        self.step_count = 0
        for i, param in enumerate(step_params):
            prev_sc = self.step_count
            while (self.step_count == prev_sc or self.step_count % self.period_steps != 0) and not self.is_stop:
                rospy.sleep(0.001)
            if self.is_stop:
                break
            self._update_walking_params(param)
            print("Set step %s param." % i)

        self.stop()

    def run(self, *args, **kwargs):
        if self.is_stop:
            Thread(target=self.run_th, args=args, kwargs=kwargs).start()

    def _read_ep_params(self, json_file, ep_i):
        with open(json_file) as f:
            data = json.load(f)
            eps = data["params_detail"]
            ramp_angle = float(list(eps[ep_i].keys())[0])
            print("ramp_angle: %.2f" % ramp_angle)
            params = list(eps[ep_i].values())[0]
        return params

    # TODO: Figure out how to transform to real robot.
    def _walk_ready(self):
        self._pub_cmd.publish("start")
        # Prepare initial state
        gyro = [self.imu.angular_velocity.x,
                self.imu.angular_velocity.y,
                self.imu.angular_velocity.z]
        accel = [self.imu.linear_acceleration.x,
                 self.imu.linear_acceleration.y,
                 self.imu.linear_acceleration.z]
        self.step_state.accel_xyz_signal.append(accel)
        self.step_state.gyro_xyz_signal.append(gyro)


def run(*args, **kwargs):
    op3 = Walk(*args, **kwargs)
    op3.run()
    # op3._update_walking_params()


if __name__ == '__main__':
    run("up_params.json", epi=0)
