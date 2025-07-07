# Juno2 Baking Helper Robot 👩🏻‍🍳🍰🥣🍪 - Group 17

This project is a ROS-based interactive assistant designed to help users follow cooking recipes using either **speech** or **object detection** to select ingredients. It includes modules for:

- 🎤 Speech-based ingredient detection
- 📷 Object detection-based ingredient identification using YOLOv8
- 👨‍🍳 Step-by-step voice-guided recipe instruction

## Project Setup 🚀

### 📌 Prerequisites

- Ubuntu 20.04 or 22.04
- ROS Noetic or ROS 2 (e.g., Foxy/Humble)
- Python 3.8+
- Microphone and camera access

### 📌 Create a ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

### 📌 Create a ROS package
```
$ cd ~/catkin_ws/src/
$ catkin_create_pkg juno2_bakinghelper roscpp rospy std_msgs
```

### 📌 Build catkin workspace
```
$ cd ..
$ catkin_make
```

### 📌 Source setup file
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## User Manual

### 📌 Clone the project repository from GitHub
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/fasyuzz/juno2_bakinghelper.git
```
### 📌 Install dependencies
```
$ pip install -r juno2_bakinghelper/requirements.txt
```
### 📌 Build catkin workspace
```
$ cd ..
$ catkin_make
```
### 📌 To launch the robot application, open 2 terminals and run the following commands:
- Terminal 1:
```
$ roscore
```
- Terminal 2:
```
$ roslaunch juno2_bakinghelper bakinghelper.launch
