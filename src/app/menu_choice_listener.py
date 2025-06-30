#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import speech_recognition as sr
import pyttsx3

def menu_choice_callback(msg):
    rospy.loginfo("Callback triggered for menu choice")
    rospy.loginfo(f"Raw message: {msg.data}")

    try:
        menus = json.loads(msg.data)
    except json.JSONDecodeError as e:
        rospy.logerr(f"Failed to parse menus JSON: {e}")
        return

    rospy.loginfo("I heard these menu suggestions:")
    for idx, menu in enumerate(menus):
        rospy.loginfo(f"{idx+1}. {menu['name']}")

    # announce the menus
    engine = pyttsx3.init()
    for idx, menu in enumerate(menus):
        say_text = f"Option {idx+1}: {menu['name']}"
        engine.say(say_text)
    engine.runAndWait()

    # initialize publisher before recognizing to avoid race conditions
    pub = rospy.Publisher('/baking_menu_choice', String, queue_size=10)

    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        rospy.loginfo("Please say your menu choice number:")
        try:
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=10)
            spoken_text = recognizer.recognize_google(audio)
            rospy.loginfo(f"Recognized speech: {spoken_text}")
        except sr.WaitTimeoutError:
            rospy.logwarn("Timeout: no speech detected, defaulting to option 1")
            spoken_text = "1"
        except sr.UnknownValueError:
            rospy.logwarn("Could not understand speech, defaulting to option 1")
            spoken_text = "1"
        except Exception as e:
            rospy.logerr(f"Speech recognition error: {e}, defaulting to option 1")
            spoken_text = "1"

    # convert word to number
    word_to_number = {
        "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
        "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10
    }

    cleaned = spoken_text.strip().lower()
    if cleaned.isdigit():
        chosen_idx = int(cleaned)
    elif cleaned in word_to_number:
        chosen_idx = word_to_number[cleaned]
    else:
        rospy.logwarn("Could not match spoken choice, defaulting to option 1")
        chosen_idx = 1

    # clamp to valid range
    chosen_idx = max(1, min(chosen_idx, len(menus)))

    confirm_text = f"You chose option {chosen_idx}. Proceeding."
    rospy.loginfo(confirm_text)
    engine.say(confirm_text)
    engine.runAndWait()

    rospy.sleep(1)
    pub.publish(str(chosen_idx))

def listener():
    rospy.init_node('menu_choice_listener')
    rospy.Subscriber('/baking_menu', String, menu_choice_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
