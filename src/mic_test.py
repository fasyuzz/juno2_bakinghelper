#!/usr/bin/env python3

import speech_recognition as sr

recognizer = sr.Recognizer()
mic = sr.Microphone()

print("Juno2 robot mic test: please speak after the beep.")

with mic as source:
    recognizer.adjust_for_ambient_noise(source)
    print("Listening...")
    audio = recognizer.listen(source)

try:
    text = recognizer.recognize_google(audio)
    print(f"Recognized: {text}")
except sr.UnknownValueError:
    print("Could not understand.")
except sr.RequestError as e:
    print(f"Request error: {e}")
