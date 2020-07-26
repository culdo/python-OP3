from darknet_video.core.thread_detector import ThreadingDetector


def hand_yolo(url, weights, **kwargs):
    ThreadingDetector(url,
                      weights_path=weights,
                      meta_file="hands.data",
                      config_path="yolov4-hands.cfg",
                      **kwargs)


if __name__ == '__main__':
    hand_weights = "backup/yolov4-hands_bak.weights"
    op3_camera = "http://127.0.0.1:8080/stream?topic=/usb_cam_node/image_raw&type=ros_compressed"
    hand_yolo(op3_camera, hand_weights, thresh=0.25, show_gui=True)
