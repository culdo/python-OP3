import json
import os
import time
from threading import Thread

import rospy
from std_msgs.msg import String


class VoiceController:
    def __init__(self, op3, forever=False):
        cmd_join = os.path.join(os.path.dirname(__file__), "voice_cmd.json")
        with open(cmd_join, encoding="utf-8") as f:
            self.voice_cmd = json.load(f)
        self.op3 = op3
        self.op3.google_stt(lang="en-US", blocking=False)
        self._pub_act = rospy.Publisher("/yolo_action", String)
        self.op3.tts_lang = "en-US"
        self.run(forever)

    def run(self, forever):
        def func():
            print("Start voice control.")
            while not rospy.is_shutdown():
                stt_result = self.op3.stt_result.lower()
                if stt_result == "come here":
                    self.op3.online_walking_command(step_time=0.5, step_num=8)
                    self.op3.stt_result = ""
                    self._after_walking()
                elif stt_result != "":
                    for action, texts in self.voice_cmd.items():
                        if stt_result in texts:
                            self.op3.play_motion(action, start_voice="Hello!", is_blocking=False)
                            self.op3.stt_result = ""
                            break
                else:
                    time.sleep(0.01)
            print("Stop voice control.")

        t = Thread(target=func)
        t.start()
        if forever:
            t.join()

    def _after_walking(self):
        self.op3.play_motion(1, start_voice=None)
        self._pub_act.publish("Start")
