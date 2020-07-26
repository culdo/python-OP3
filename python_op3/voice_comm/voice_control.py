import json
from threading import Thread

import os


class VoiceController:
    def __init__(self, op3, forever=False):
        cmd_join = os.path.join(os.path.dirname(__file__), "voice_cmd.json")
        with open(cmd_join, encoding="utf-8") as f:
            self.voice_cmd = json.load(f)
        self.op3 = op3
        self.op3.google_stt(lang="en-US", blocking=False)
        self.stop = False
        self.op3.tts_lang = "en-US"
        self.run(forever)

    def run(self, forever):
        def func():
            print("Start voice control.")
            while not self.stop:
                stt_result = self.op3.stt_result.lower()
                if stt_result == "come here":
                    self.op3.online_walking_command()
                else:
                    for action, texts in self.voice_cmd.items():
                        if stt_result in texts:
                            self.op3.play_motion(action, done_voice="I'm done.", is_blocking=False)
                            break
                self.op3.stt_result = ""
            print("Stop voice control.")
        t = Thread(target=func)
        t.start()
        if forever:
            t.join()

    def stop(self):
        self.stop = True


