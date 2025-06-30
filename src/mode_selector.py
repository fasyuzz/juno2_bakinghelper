#!/usr/bin/env python3

import subprocess

def main():
    print("""
===========================================
  Welcome to Juno2 Baking Helper
===========================================
Please choose an option:
1. Speech-based interaction
2. Object-based interaction
q. Quit
""")

    while True:
        choice = input("Enter your choice (1/2/q): ").strip().lower()
        if choice == "1":
            print("Starting speech-based mode...")
            subprocess.call(["rosrun", "juno2_baking_helper", "baking_helper_node.py"])
            break
        elif choice == "2":
            print("Starting object-based mode...")

            # start the usb_cam in a new terminal
            subprocess.Popen(
                ["gnome-terminal", "--", "roslaunch", "usb_cam", "usb_cam-test.launch"]
            )

            # small pause to let camera launch
            import time
            time.sleep(3)

            # then start the object detection
            subprocess.call(["rosrun", "juno2_baking_helper", "baking_helper_object_node.py"])
            break
        elif choice == "q":
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1, 2, or q.")

if __name__ == "__main__":
    main()

