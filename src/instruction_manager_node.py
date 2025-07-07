#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from app.instruction_manager import InstructionManager

def instruction_manager_node():
    rospy.init_node("instruction_manager_node")

    def callback(msg):
        try:
            recipe = json.loads(msg.data)
            rospy.loginfo(f"[instruction_manager] Received recipe: {recipe['name']}")
        except Exception as e:
            rospy.logerr(f"JSON parse error: {e}")
            return

        instr = InstructionManager(recipe)
        instr.start()

    rospy.Subscriber("/suggested_recipe", String, callback)

    rospy.loginfo("instruction_manager_node running, waiting for suggested recipes...")
    rospy.spin()

if __name__ == "__main__":
    try:
        instruction_manager_node()
    except rospy.ROSInterruptException:
        pass

