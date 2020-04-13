from threading import Thread

import rospy
from std_msgs.msg import String
import os
from gtts import gTTS
import speech_recognition as sr
from http.client import BadStatusLine


class Utility(object):
    def __init__(self):
        # Utility
        self._pub_sound = rospy.Publisher("/play_sound_file", String, queue_size=0)
        self.r = sr.Recognizer()
        self.desc = None

    def google_tts(self, query="我是TTSAPI", lang="zh-TW"):
        tts = gTTS(query, lang=lang)
        audio_file = 'buff.mp3'
        tts.save(audio_file)
        self._pub_sound.publish(os.path.realpath(audio_file))

    def google_stt(self, lang="zh-TW", blocking=True):
        def thread():
            while True:
                with sr.Microphone(device_index=4) as source:
                    print("Say something")
                    audio = self.r.listen(source)
                    print('time over')
                try:
                    self.desc = self.r.recognize_google(audio, language=lang)
                    print('text: %s' % self.desc)
                except (sr.UnknownValueError, sr.RequestError, BadStatusLine) as e:
                    print(type(e).__name__)
                    if "Too Many Requests" in str(e):
                        raise ConnectionRefusedError("Too Many Requests")
        t = Thread(target=thread)
        t.start()
        if blocking:
            t.join()

