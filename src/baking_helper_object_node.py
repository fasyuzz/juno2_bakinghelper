#!/usr/bin/env python3

import rospy
import pyttsx3
import json
import os
from app.object_detection import IngredientDetector
from app.menu_suggester import suggest_menus
from app.instruction_manager import InstructionManager
from app.my_speech_recognition import recognize_speech
from std_msgs.msg import String

# recipe file path
recipe_file = os.path.join(os.path.dirname(__file__), "models/recipe_database.json")

def load_all_recipes():
    try:
        with open(recipe_file, "r") as f:
            recipes = json.load(f)
        return recipes
    except Exception as e:
        rospy.logerr(f"Failed to load recipe database: {e}")
        return []

def baking_helper_object():
    rospy.init_node("baking_helper_object_node")
    engine = pyttsx3.init()
    engine.setProperty("rate", engine.getProperty("rate") - 50)

    detector = IngredientDetector()

    # publisher
    ingredient_pub = rospy.Publisher("/detected_ingredients", String, queue_size=10)

    all_detected_ingredients = set()

    while not rospy.is_shutdown():
        rospy.loginfo("Juno2: Please show the ingredients to the camera.")
        engine.say("Please show me the ingredients.")
        engine.runAndWait()

        user_input = input("Juno2: Press ENTER to capture, 'p' to proceed, or 'q' to quit: ").strip().lower()

        if user_input == "q":
            rospy.loginfo("Juno2: Exiting the system. Goodbye!")
            engine.say("Exiting the system. Goodbye!")
            engine.runAndWait()
            break

        elif user_input == "p":
            if not all_detected_ingredients:
                rospy.logwarn("No ingredients detected yet.")
                engine.say("No ingredients detected yet. Please capture first.")
                engine.runAndWait()
                continue
            rospy.loginfo("Proceeding with detected ingredients...")
            engine.say("Proceeding with the detected ingredients.")
            engine.runAndWait()
        else:
            detector.capture_and_detect()
            detected = detector.get_detected()
            rospy.loginfo(f"Captured ingredients: {detected}")

            # accumulate
            all_detected_ingredients.update(detected)

            # publish immediately
            ingredient_pub.publish(json.dumps(list(all_detected_ingredients)))
            rospy.loginfo(f"Published detected ingredients: {list(all_detected_ingredients)}")

            continue  # go back to capture loop

        # after user presses 'p', proceed to menu
        menus = suggest_menus(list(all_detected_ingredients))
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

        # publish detected ingredients again before starting
        ingredient_pub.publish(json.dumps(list(all_detected_ingredients)))

        instr_mgr = InstructionManager(chosen_recipe)
        instr_mgr.start()

if __name__ == "__main__":
    try:
        baking_helper_object()
    except rospy.ROSInterruptException:
        pass


# import rospy
# import pyttsx3
# import json
# import os
# import time
# from std_msgs.msg import String
# from app.object_detection import IngredientDetector
# from app.menu_suggester import suggest_menus
# from app.instruction_manager import InstructionManager
# from app.my_speech_recognition import recognize_speech

# recipe_file = os.path.join(os.path.dirname(__file__), "models/recipe_database.json")

# def load_all_recipes():
#     try:
#         with open(recipe_file, "r") as f:
#             recipes = json.load(f)
#         return recipes
#     except Exception as e:
#         rospy.logerr(f"Failed to load recipe database: {e}")
#         return []

# def baking_helper_object():
#     rospy.init_node("baking_helper_object_node")
#     engine = pyttsx3.init()
#     engine.setProperty("rate", engine.getProperty("rate") - 50)

#     detector = IngredientDetector()
#     all_detected_ingredients = set()

#     # publisher
#     ingredient_pub = rospy.Publisher("/detected_ingredients", String, queue_size=10)

#     while not rospy.is_shutdown():
#         rospy.loginfo("Juno2: Please show the ingredients to the camera.")
#         engine.say("Please show the ingredients to the camera.")
#         engine.runAndWait()

#         user_input = input("Juno2: When you are ready, press ENTER to capture, 'p' to proceed, or 'q' to quit: ").strip().lower()

#         if user_input == "q":
#             rospy.loginfo("Juno2: Exiting the system. Goodbye!")
#             engine.say("Exiting the system. Goodbye!")
#             engine.runAndWait()
#             break

#         elif user_input == "p":
#             if not all_detected_ingredients:
#                 rospy.logwarn("No ingredients captured yet. Please capture first.")
#                 engine.say("I have not detected any ingredients yet. Please capture first.")
#                 engine.runAndWait()
#                 continue
#             rospy.loginfo("Juno2: Proceeding with the detected ingredients.")
#             engine.say("Proceeding with the detected ingredients.")
#             engine.runAndWait()

#             # publish once before proceeding
#             ingredient_pub.publish(json.dumps(list(all_detected_ingredients)))

#             ingredients_list = list(all_detected_ingredients)
#             menus = suggest_menus(ingredients_list)
#             if not menus:
#                 rospy.loginfo("No matching menus found, using all available recipes instead.")
#                 menus = load_all_recipes()

#             if not menus:
#                 rospy.logwarn("No menus available. Please check your recipe database.")
#                 engine.say("Sorry, I cannot find any recipes.")
#                 engine.runAndWait()
#                 continue

#             rospy.loginfo("Suggested menus:")
#             for idx, menu in enumerate(menus):
#                 rospy.loginfo(f"{idx+1}. {menu['name']}")
#                 engine.say(f"Option {idx+1}: {menu['name']}")
#                 engine.runAndWait()

#             engine.say("Please say your menu choice number.")
#             engine.runAndWait()

#             choice_text = recognize_speech()
#             rospy.loginfo(f"Recognized choice: {choice_text}")

#             word_to_number = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}
#             cleaned = choice_text.strip().lower()
#             if cleaned.isdigit():
#                 chosen_idx = int(cleaned)
#             elif cleaned in word_to_number:
#                 chosen_idx = word_to_number[cleaned]
#             else:
#                 rospy.loginfo("Could not match spoken choice, defaulting to option 1")
#                 chosen_idx = 1

#             chosen_idx = max(1, min(chosen_idx, len(menus)))
#             chosen_recipe = menus[chosen_idx - 1]

#             confirm_text = f"You chose option {chosen_idx}: {chosen_recipe['name']}. I will now start guiding you."
#             rospy.loginfo(confirm_text)
#             engine.say(confirm_text)
#             engine.runAndWait()

#             instr_mgr = InstructionManager(chosen_recipe)
#             instr_mgr.start()
#             break  # finish after instructions

#         else:
#             # user pressed ENTER
#             detector.capture_and_detect()
#             current_detected = detector.get_detected()
#             all_detected_ingredients.update(current_detected)
#             rospy.loginfo(f"Juno2: Collected ingredients so far: {list(all_detected_ingredients)}")
#             # publish every capture
#             ingredient_pub.publish(json.dumps(list(all_detected_ingredients)))
#             rospy.loginfo("Published detected ingredients to /detected_ingredients")

# if __name__ == "__main__":
#     try:
#         baking_helper_object()
#     except rospy.ROSInterruptException:
#         pass
