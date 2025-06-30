import pyttsx3
import rospy
from app.my_speech_recognition import recognize_speech

class InstructionManager:
    def __init__(self, recipe):
        self.recipe = recipe
        self.steps = recipe.get("steps", [])
        self.engine = pyttsx3.init()
        rate = self.engine.getProperty('rate')
        self.engine.setProperty('rate', rate - 50) #adjust speed speech

    def start(self):
        if not self.steps:
            self.engine.say("This recipe has no steps.")
            self.engine.runAndWait()
            return

        self.engine.say(f"Starting the recipe for {self.recipe['name']}.")
        self.engine.runAndWait()

        idx = 0
        while idx < len(self.steps):
            step_text = f"Step {idx+1}: {self.steps[idx]}"
            rospy.loginfo(step_text)
            self.engine.say(step_text)
            self.engine.runAndWait()

            while True:
                self.engine.say("Please say next, repeat, or stop.")
                self.engine.runAndWait()
                #command = recognize_speech().lower().strip()
                command = recognize_speech(timeout=5, phrase_time_limit=5)
                rospy.loginfo(f"User said: {command}")
                if "stop" in command:
                    self.engine.say("Okay, ending the recipe guidance. Enjoy your baking!")
                    self.engine.runAndWait()
                    rospy.signal_shutdown("Recipe being cut")
                    return
                elif "next" in command:
                    idx += 1
                    break
                elif "repeat" in command:
                    rospy.loginfo(step_text)
                    self.engine.say(step_text)
                    self.engine.runAndWait()
                else:
                    self.engine.say("Sorry, I did not understand.")
                    self.engine.runAndWait()
        self.engine.say("All steps are done. Enjoy your baking!")
        self.engine.runAndWait()
        rospy.signal_shutdown("Recipe complete")

