from sensor_msgs.msg import Imu
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
import rospy


class OpenCR(object):
    def __init__(self, ns):
        self._sub_imu = rospy.Subscriber(ns + "/open_cr/imu", Imu, self._cb_imu, queue_size=10)
        self._sub_button = rospy.Subscriber(ns + "/open_cr/button", String, self._cb_button, queue_size=10)

    def _cb_imu(self, msg):
        # print msg
        pass

    def _cb_button(self, msg):
        """
        custom callback
        :data: "mode", "start", "user"
        (when long-press, data=data+"_long")
        :return:
        """
        # print msg
        pass

    def set_led(self, led):
        """
        :param led: 1, 2, 4, 7(ALL)
        :return:
        """
        msg = SyncWriteItem()
        msg.item_name = "LED"
        msg.joint_name = ["open-cr"]
        msg.value = [led]

        self._pub_sync_write.publish(msg)
