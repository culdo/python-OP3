from threading import Thread

import rospy
from std_msgs.msg import String
import os
from gtts import gTTS
import speech_recognition as sr
from http.client import BadStatusLine


class Utility(object):
    def __init__(self, tts_lang="zh-TW"):
        # Utility
        self.tts_lang = tts_lang
        self._pub_sound = rospy.Publisher("/play_sound_file", String, queue_size=0)
        self.r = sr.Recognizer()
        self.stt_result = ""

    def google_tts(self, query="我是TTSAPI"):
        tts = gTTS(query, lang=self.tts_lang)
        audio_file = 'buff.mp3'
        tts.save(audio_file)
        self._pub_sound.publish(os.path.realpath(audio_file))

    def record_speech(self, file_name, echo=False):
        with open(file_name, 'wb') as f:
            f.write(self.audio.get_wav_data())
        if echo:
            self._pub_sound.publish(os.path.realpath(file_name))

    def google_stt(self, lang="zh-TW", blocking=False):
        def thread():
            while not rospy.is_shutdown():
                with sr.Microphone(device_index=4) as source:
                    self.r.adjust_for_ambient_noise(source)
                    print("Say something")
                    self.audio = self.r.listen(source)
                    self.record_speech("speech.wav", echo=True)
                    print('listened')
                try:
                    self.stt_result = self.r.recognize_google(self.audio, language=lang)
                    print('text: %s' % self.stt_result)
                except Exception as e:
                    print(type(e).__name__)
                    if "Too Many Requests" in str(e):
                        raise ConnectionRefusedError("Too Many Requests")
        t = Thread(target=thread, daemon=True)
        t.start()
        if blocking:
            t.join()

