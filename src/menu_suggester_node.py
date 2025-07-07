#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
import pyttsx3
from app.menu_suggester import suggest_menus

def menu_suggester_node():
    rospy.init_node("menu_suggester_node")
    pub = rospy.Publisher("/suggested_recipe", String, queue_size=10)
    engine = pyttsx3.init()
    engine.setProperty("rate", engine.getProperty("rate") - 50)

    def callback(msg):
        try:
            ingredients = json.loads(msg.data)
            rospy.loginfo(f"[menu_suggester] Received ingredients: {ingredients}")
        except Exception as e:
            rospy.logerr(f"[menu_suggester] JSON parse error: {e}")
            return

        menus = suggest_menus(ingredients)
        if not menus:
            rospy.logwarn("No matching menus found.")
            engine.say("Sorry, I could not find any matching recipes.")
            engine.runAndWait()
            return
        
        rospy.loginfo("Suggested menus:")
        for idx, menu in enumerate(menus):
            rospy.loginfo(f"{idx+1}. {menu['name']}")
            engine.say(f"Option {idx+1}: {menu['name']}")
            engine.runAndWait()
        
        engine.say("Please say the option number to choose.")
        engine.runAndWait()

        # send the first suggestion by default to instruction manager
        chosen_recipe_json = json.dumps(menus[0])
        pub.publish(chosen_recipe_json)
        rospy.loginfo(f"Published chosen recipe: {menus[0]['name']}")

    rospy.Subscriber("/user_ingredients", String, callback)
    rospy.Subscriber("/detected_ingredients", String, callback)

    rospy.loginfo("menu_suggester_node running, waiting for ingredients...")
    rospy.spin()

if __name__ == "__main__":
    try:
        menu_suggester_node()
    except rospy.ROSInterruptException:
        pass
