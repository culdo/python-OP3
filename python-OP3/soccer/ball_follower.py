import rospy
from math import pi, tan
from geometry_msgs.msg import Point
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam


class BallFollower(object):
    def __init__(self):
        self.ball_position = Point()
        self.current_walking_param_ = WalkingParam()

        self.FOV_WIDTH = 35.2 * pi / 180
        self.FOV_HEIGHT = 21.6 * pi / 180
        self.count_not_found_ = 0
        self.count_to_kick_ = 0
        self.on_tracking_ = False
        self.approach_ball_position_ = "NotFound"
        self.kick_motion_index_ = 83
        self.CAMERA_HEIGHT = 0.46
        self.NOT_FOUND_THRESHOLD = 50
        self.MAX_FB_STEP = 40.0 * 0.001
        self.MAX_RL_TURN = 15.0 * pi / 180
        self.IN_PLACE_FB_STEP = -3.0 * 0.001
        self.MIN_FB_STEP = 5.0 * 0.001
        self.MIN_RL_TURN = 5.0 * pi / 180
        self.UNIT_FB_STEP = 1.0 * 0.001
        self.UNIT_RL_TURN = 0.5 * pi / 180
        self.SPOT_FB_OFFSET = 0.0 * 0.001
        self.SPOT_RL_OFFSET = 0.0 * 0.001
        self.SPOT_ANGLE_OFFSET = 0.0
        self.hip_pitch_offset_ = 7.0
        self.current_pan_ = -10
        self.current_tilt_ = -10
        self.current_x_move_ = 0.005
        self.current_r_angle_ = 0
        self.curr_period_time_ = 0.6
        self.accum_period_time_ = 0.0
        self.DEBUG_PRINT = False

        self.prev_time_ = rospy.get_rostime()

    def start_following(self):
        self.on_tracking_ = True
        rospy.loginfo("Start Ball following")

        self.set_walking_command("start")
        result = self.get_walking_param()
        if result:
            self.hip_pitch_offset_ = self.current_walking_param_.hip_pitch_offset
            self.curr_period_time_ = self.current_walking_param_.period_time
        else:
            self.hip_pitch_offset_ = 7.0 * pi / 180
            self.curr_period_time_ = 0.6

    def stop_following(self):
        self.on_tracking_ = False
        self.count_to_kick_ = 0

        rospy.loginfo("Stop Ball following")
        self.set_walking_command("stop")

    def calc_footstep(self, target_distance, target_angle, delta_time):
        # calc fb
        rl_angle = 0.0

        next_movement = self.current_x_move_
        if target_distance < 0:
            target_distance = 0.0
        fb_goal = min(target_distance * 0.1, self.MAX_FB_STEP)
        self.accum_period_time_ += delta_time
        if self.accum_period_time_ > (self.curr_period_time_ / 4):
            self.accum_period_time_ = 0.0
            if (target_distance * 0.1 / 2) < self.current_x_move_:
                next_movement -= self.UNIT_FB_STEP
            else:
                next_movement += self.UNIT_FB_STEP
        fb_goal = min(next_movement, fb_goal)
        fb_move = max(fb_goal, self.MIN_FB_STEP)
        rospy.loginfo("distance to ball : %6.4f, fb : %6.4f, delta : %6.6f" % (target_distance, fb_move, delta_time))
        rospy.loginfo("============================================")

        # calc rl angle
        if abs(target_angle) * 180 / pi > 5.0:
            rl_offset = abs(target_angle) * 0.2
            rl_goal = min(rl_offset, self.MAX_RL_TURN)
            rl_goal = max(rl_goal, self.MIN_RL_TURN)
            rl_angle = min(abs(self.current_r_angle_) + self.UNIT_RL_TURN, rl_goal)

            if target_angle < 0:
                rl_angle *= -1

        return fb_move, rl_angle

    def process_following(self, x_angle, y_angle, ball_size):
        curr_time = rospy.get_rostime()
        dur = curr_time - self.prev_time_

        delta_time = dur.nsecs * 0.000000001 + dur.secs
        self.prev_time_ = curr_time

        self.count_not_found_ = 0

        if self.current_tilt_ == -1 and self.current_pan_ == -10:
            rospy.logerr("Failed to get current angle of head joints.")
            self.set_walking_command("stop")
            self.on_tracking_ = False
            self.approach_ball_position_ = "NotFound"
            return False
        rospy.loginfo("   ============== Head | Ball ==============   ")
        rospy.loginfo("== Head Pan : " + (self.current_pan_ * 180 / pi) + " | Ball X : " + (x_angle * 180 / pi))
        rospy.loginfo("== Head Tilt : " + (self.current_pan_ * 180 / pi) + " | Ball Y : " + (y_angle * 180 / pi))
        self.approach_ball_position_ = "OutOfRange"

        distance_to_ball = self.CAMERA_HEIGHT * tan(pi * 0.5 + self.current_tilt_ - self.hip_pitch_offset_ - ball_size)

        ball_y_angle = (self.current_tilt_ + y_angle) * 180 / pi
        ball_x_angle = (self.current_pan_ + x_angle) * 180 / pi

        if distance_to_ball < 0:
            distance_to_ball *= -1
        distance_to_kick = 0.22
        # Kick Ball
        # if distance_to_ball<distance_to_kick and abs(ball_x_angle) <25.0:
        #     self.count_to_kick_ += 1
        #     rospy.loginfo("== Head Pan : " + (self.current_pan_ * 180 / pi) + " | Ball X : " + (ball_x_angle * 180 / pi))
        #     rospy.loginfo("== Head Tilt : " + (self.current_pan_ * 180 / pi) + " | Ball Y : " + (ball_y_angle * 180 / pi))
        #     rospy.loginfo("foot to kick : "+ball_x_angle)
        #
        # rospy.loginfo("In range [%d | %d]" % (self.count_to_kick_, ball_x_angle))
        # if self.count_to_kick_>20:
        #     self.set_walking_cmd("stop")
        #     self.on_tracking_ = False
        #     if ball_x_angle>0:
        #         rospy.loginfo("Read")
        distance_to_walk = distance_to_ball - distance_to_kick
        fb_move, rl_angle = self.calc_footstep(distance_to_walk, self.current_pan_, delta_time)

        self.set_walking_param(fb_move, 0, rl_angle)

        return False

    def decide_ball_pos(self, x_angle, y_angle):
        if self.current_tilt_ == -10 and self.current_pan_ == -10:
            self.approach_ball_position_ = "NotFound"
            return

        ball_x_angle = self.current_pan_ + x_angle
        if ball_x_angle > 0:
            self.approach_ball_position_ = "OnLeft"
        else:
            self.approach_ball_position_ = "OnRight"

    def wait_following(self):
        self.count_not_found_ += 1
        if self.count_not_found_ > self.NOT_FOUND_THRESHOLD / 2.0:
            self.set_walking_param(0.0, 0.0, 0.0)

    def set_walking_command(self, command):
        if command == "start":
            # get initial param
            self.get_walking_param()
            self.set_walking_param(self.IN_PLACE_FB_STEP, 0, 0, True)

        self._pub_walking_command.publish(command)
        rospy.loginfo("Send Walking command : " + command)

    def set_walking_param(self, x_move, y_move, rotation_angle, balance=True):
        self.current_walking_param_.balance_enable = balance
        self.current_walking_param_.x_move_amplitude = x_move + self.SPOT_FB_OFFSET
        self.current_walking_param_.y_move_amplitude = y_move + self.SPOT_RL_OFFSET
        self.current_walking_param_.angle_move_amplitude = rotation_angle + self.SPOT_ANGLE_OFFSET

        self._pub_walking_param.publish(self.current_walking_param_)

        # update variable silently
        self.current_x_move_ = x_move
        self.current_r_angle_ = rotation_angle

    def get_walking_param(self):
        walking_param_msg = GetWalkingParam()
        if self.get_walking_param_srv_(walking_param_msg):

            self.current_walking_param_ = walking_param_msg.response.parameters

            # update ui
            rospy.loginfo("Get walking parameters")

            return True
        else:
            rospy.logerr("Fail to get walking parameters.")

            return False
