#!/usr/bin/env python
import roslib; roslib.load_manifest('fleet_control')
import speech_recognition as sr
import IPython
import rospy
import string
from std_msgs.msg import String

r = sr.Recognizer()
r.energy_threshold = 4000
#r.pause_threshold = 0.501

def getSpeech():
    with sr.Microphone() as source: # use the default microphone as the audio source
        audio = r.listen(source) # listen for the first phrase and extract it into audio data

        try:
            rec = r.recognize(audio, True)
            print rec
            return rec # recognize speech using Google Speech Recognition
        except LookupError: # speech is unintelligible
            print("Could not understand audio")
            return None

pub = rospy.Publisher('/dynamic_nav/voice_cmd', String, queue_size=1)
rospy.init_node('speech_node')

while True:
    data = getSpeech()
    print data
    if data is None:
        continue
    for elem in data:
        speech = string.lower(elem['text'])
        if 'stop' in speech:
            pub.publish(String(data='stop'))
            break
        elif 'continue' in speech:
            pub.publish(String(data='go'))
            break
    #else ignore it
