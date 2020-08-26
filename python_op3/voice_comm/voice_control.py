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
        self.walking = False

    def run(self, forever):
        self.json_f, self.ep_i = None, None
        def func():
            print("Start voice control.")
            while not rospy.is_shutdown():
                stt_result = self.op3.stt_result.lower()
                if stt_result != "":
                    if stt_result == "get over it":
                        self.walking = True
                    if self.walking:
                        if stt_result == "up":
                            self.json_f = "up_params.json"
                        elif stt_result == "down":
                            self.json_f = "down_params.json"
                        elif stt_result in ["one", "1"]:
                            self.ep_i = 1
                        elif stt_result in ["two", "2"]:
                            self.ep_i = 2
                        elif stt_result in ["three", "3"]:
                            self.ep_i = 3
                        if self.json_f and self.ep_i:
                            self.op3.google_tts("Walk %s on episode %s" % (self.json_f, self.ep_i))
                            self.op3.walker.run(self.json_f, self.ep_i)
                            self.json_f, self.ep_i = None, None
                    else:
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
