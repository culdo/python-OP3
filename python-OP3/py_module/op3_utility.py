# -*- coding: utf-8 -*-
import rospy
import requests
from std_msgs.msg import String
import os


class Utility(object):
    def __init__(self):
        # Utility
        self._pub_sound = rospy.Publisher("/play_sound_file", String, queue_size=0)

    def google_tts(self, query="我是TTSAPI", lang="zh"):
        doc = requests.get(
            "https://translate.google.com/translate_tts?ie=UTF-8&q=" + query + "&tl=" + lang + "&client=tw-ob")
        with open('buff.mp3', 'wb') as f:
            f.write(doc.content)
            fpath = f.name
        self._pub_sound.publish(os.path.realpath(fpath))
