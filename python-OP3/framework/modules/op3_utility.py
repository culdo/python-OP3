import rospy
import requests
from std_msgs.msg import String
import os
from gtts import gTTS


class Utility(object):
    def __init__(self):
        # Utility
        self._pub_sound = rospy.Publisher("/play_sound_file", String, queue_size=0)

    def google_tts(self, query="我是TTSAPI", lang="zh-TW"):
        tts = gTTS(query, lang=lang)
        audio_file = 'buff.mp3'
        tts.save(audio_file)
        self._pub_sound.publish(os.path.realpath(audio_file))
