#!/usr/bin/env python
import speech_recognition as sr

def get_input():
    r = sr.Recognizer()
    with sr.Microphone() as source: # use the default microphone as the audio source
        audio = r.listen(source) # listen for the first phrase and extract it into audio data

        try:
            text = r.recognize(audio) # recognize speech using Google Speech Recognition
            return text
        except LookupError: # speech is unintelligible
            print("Could not understand audio")
            return ""
            
print get_input()
