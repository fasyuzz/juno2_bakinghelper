#!/usr/bin/env python3

import speech_recognition as sr
import rospy

def recognize_speech(timeout=10, phrase_time_limit=10):
    recognizer = sr.Recognizer()
    try:
        with sr.Microphone() as source:
            rospy.loginfo("Say something:")
            try:
                audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
                rospy.loginfo("Captured audio, recognizing...")
            except sr.WaitTimeoutError:
                rospy.logwarn("Timeout: no speech detected")
                return ""
        try:
            text = recognizer.recognize_google(audio)
            rospy.loginfo(f"You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            rospy.logwarn("Sorry, could not understand.")
            return ""
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results: {e}")
            return ""
    except Exception as e:
        rospy.logerr(f"Microphone error: {e}")
        return ""

