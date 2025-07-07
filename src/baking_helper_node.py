#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import pyttsx3
import json
import os

from app.my_speech_recognition import recognize_speech
from app.ingredient_parser import parse_ingredients
from app.menu_suggester import suggest_menus
from app.instruction_manager import InstructionManager

# recipe file
recipe_file = os.path.join(os.path.dirname(__file__), "../models/recipe_database.json")

def load_all_recipes():
    try:
        with open(recipe_file, "r") as f:
            recipes = json.load(f)
        return recipes
    except Exception as e:
        rospy.logerr(f"Failed to load recipe database: {e}")
        return []

def baking_helper():
    rospy.init_node("baking_helper_speech_node")
    engine = pyttsx3.init()
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)

    # define publisher
    pub = rospy.Publisher("/user_ingredients", String, queue_size=10)

    while not rospy.is_shutdown():
        rospy.loginfo("Juno2: Please speak the ingredients.")
        engine.say("Please tell me the ingredients you have.")
        engine.runAndWait()

        spoken_text = recognize_speech()
        rospy.loginfo(f"Recognized: {spoken_text}")

        ingredients = parse_ingredients(spoken_text)
        rospy.loginfo(f"Parsed ingredients: {ingredients}")

        # publish the recognized ingredients
        pub.publish(json.dumps(ingredients))
        rospy.loginfo(f"Published ingredients to /user_ingredients: {ingredients}")

        menus = suggest_menus(ingredients)
        if not menus:
            rospy.loginfo("No matching menus found, using all available recipes instead.")
            menus = load_all_recipes()

        if not menus:
            rospy.logwarn("No menus available. Please check your recipe database.")
            engine.say("Sorry, I cannot find any recipes.")
            engine.runAndWait()
            continue

        rospy.loginfo("Suggested menus:")
        for idx, menu in enumerate(menus):
            rospy.loginfo(f"{idx+1}. {menu['name']}")
            engine.say(f"Option {idx+1}: {menu['name']}")
            engine.runAndWait()

        engine.say("Please say your menu choice number.")
        engine.runAndWait()

        choice_text = recognize_speech()
        rospy.loginfo(f"Recognized choice: {choice_text}")

        word_to_number = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}
        cleaned = choice_text.strip().lower()
        if cleaned.isdigit():
            chosen_idx = int(cleaned)
        elif cleaned in word_to_number:
            chosen_idx = word_to_number[cleaned]
        else:
            rospy.loginfo("Could not match spoken choice, defaulting to option 1")
            chosen_idx = 1

        chosen_idx = max(1, min(chosen_idx, len(menus)))
        chosen_recipe = menus[chosen_idx - 1]

        confirm_text = f"You chose option {chosen_idx}: {chosen_recipe['name']}. I will now start guiding you."
        rospy.loginfo(confirm_text)
        engine.say(confirm_text)
        engine.runAndWait()

        instr_mgr = InstructionManager(chosen_recipe)
        instr_mgr.start()

if __name__ == "__main__":
    try:
        baking_helper()
    except rospy.ROSInterruptException:
        pass

