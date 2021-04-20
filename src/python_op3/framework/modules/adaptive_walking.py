#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Bool

from python_op3.framework.modules.walking import Walk

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


class AdaptiveWalk(Walk):
    def __init__(self):

        super().__init__()

        self._pub_state = rospy.Publisher("/adaptive_walker/imu_data", Float32MultiArray)
        self._pub_fallen = rospy.Publisher("/adaptive_walker/fallen", Bool)

        self.acc_l = []
        self.gyro_l = []

    def pending_64(self):
        acc_l = []
        gyro_l = []
        while len(acc_l) < 64:
            if self.step_count % 2 == 0:
                acc = self.imu.linear_acceleration
                acc_l.append([acc.x, acc.y, acc.z])
                gyro = self.imu.angular_velocity
                gyro_l.append([gyro.x, gyro.y, gyro.z])
        state = np.array([acc_l, gyro_l])
        return state

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
        while not self.is_stop:
            prev_sc = self.step_count
            if prev_sc != self.step_count:
                if len(self.acc_l) < 64:
                    if self.step_count % 2 == 0:
                        acc = self.imu.linear_acceleration
                        self.acc_l.append([acc.x, acc.y, acc.z])
                        gyro = self.imu.angular_velocity
                        self.gyro_l.append([gyro.x, gyro.y, gyro.z])
                    else:
                        rospy.sleep(0.01)
                else:
                    param = self.get_ddpg_predict()
                    self._update_walking_params(param)
            else:
                rospy.sleep(0.001)

        self.stop()

    def get_ddpg_predict(self):
        state = [self.acc_l, self.gyro_l]
        self._pub_state.publish(state)


def run(*args, **kwargs):
    op3 = AdaptiveWalk(*args, **kwargs)
    op3.run()
    # op3._update_walking_params()


if __name__ == '__main__':
    run("up_params.json", epi=0)
