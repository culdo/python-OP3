from threading import Thread

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import cv2


class UsbCam(object):
    def __init__(self):
        self._sub_joints = rospy.Subscriber("/usb_cam_node/image_raw/compressed",
                                            CompressedImage, self._cb_img, queue_size=10)

    def _cb_img(self, msg):
        self.img = cv2.imdecode(np.fromstring(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def display_img(self):
        def thread():
            while True:
                cv2.imshow("usb_cam_topic", self.img)
                if cv2.waitKey(1) == ord('q'):
                    break
        cv_win = Thread(target=thread)
        cv_win.start()
        cv_win.join()
