#!/usr/bin/env python3

#!/usr/bin/env python3

import subprocess
import time

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

            # open menu_suggester in a new terminal
            subprocess.Popen(
                ["gnome-terminal", "--", "rosrun", "juno2_baking_helper", "menu_suggester_node.py"]
            )
            time.sleep(1)

            # open instruction_manager in new terminal
            subprocess.Popen(
                ["gnome-terminal", "--", "rosrun", "juno2_baking_helper", "instruction_manager_node.py"]
            )
            time.sleep(1)

            # finally run baking_helper_node in current terminal
            subprocess.call(
                ["rosrun", "juno2_baking_helper", "baking_helper_node.py"]
            )
            break

        elif choice == "2":
            print("Starting object-based mode...")

            # open usb_cam
            subprocess.Popen(
                ["gnome-terminal", "--", "roslaunch", "usb_cam", "usb_cam-test.launch"]
            )
            time.sleep(3)

            # open menu_suggester
            subprocess.Popen(
                ["gnome-terminal", "--", "rosrun", "juno2_baking_helper", "menu_suggester_node.py"]
            )
            time.sleep(1)

            # open instruction_manager
            subprocess.Popen(
                ["gnome-terminal", "--", "rosrun", "juno2_baking_helper", "instruction_manager_node.py"]
            )
            time.sleep(1)

            # run baking_helper_object_node in current terminal
            subprocess.call(
                ["rosrun", "juno2_baking_helper", "baking_helper_object_node.py"]
            )
            break

        elif choice == "q":
            print("Goodbye!")
            break

        else:
            print("Invalid choice. Please enter 1, 2, or q.")

if __name__ == "__main__":
    main()


