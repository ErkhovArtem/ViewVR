# ViewVR: Visual-Feedback-Modes-to-Achieve-Quality-of-VR-based-Telemanipulation

This repo contains code and other supplementary materials for the paper **ViewVR: Visual-Feedback-Modes-to-Achieve-Quality-of-VR-based-Telemanipulation** submitted to IEEE International Conference on Human-Robot Interaction 2025 (HRI).

The paper focuses on an immersive teleoperation system that enhance operators’ ability to actively perceive the robot’s surroundings. A consumer-grade HTC Vive VR system was utilized to synchronize the operator’s and and head movements with a UR3 robot and a custom-built robotic head with two degrees of freedom (2-DoF). The system’s usability, manipulation efficiency, and intuitiveness of control were evaluated in comparison
with static head camera positioning across three distinct tasks.

<!-- GETTING STARTED -->
## Getting Started

Project consists of three main parts, divided into folders:

`Robotic_head` folder contains source code to control our custom-built robotic head.

`Unity` folder contains unity project for streaming video from webcameras into VR headset. You can find all information about installation and usage inside the folder.

`Teleoperation` folder is the main part of the project that is running on operator's PC. Below you will find instrutions only for this part.

### Hardware requirements

We assume that you have UR3 robot with Robotiq 2F-85 gripper. In our setup we used custom gripper control device, but you can implement your device after making the necessary changes to the code, described below.

`HTC VIVE PRO VR` system was used to obtain positions of operators head and hand and to provide visual feedback. 

To get visual feedback into your VR glasses, you need to connect one or more webcams via USB to your PC.

The most problematic part for implementation is robotic head, that was build by us for this project. You can test the rest part of our system without any problems, because robot and head control pipelines are fully independent.

### Installation

1. Clone this repo on your PC. 

2. Install Unity project according instructions in subdirectory.

3. Move to `teleoperation` folder and install dependensies:

````sh
cd teleoperation
pip install -r requirements.txt
````

4. Modify `config.json` according to your hardware. Here you can find instructions how to get necessary data: https://pypi.org/project/vive-tracker-apiserver/

5. To control a gripper with your hardware, overwrite `read_pose` function in file `teleop_lip.py`. This function should return number in range 0..255 that will be mapped to gripper position.

6. In file `teleop_ur.py` change variables `hand_tracker` and `robot_ip` according to your setup.

7. You would like also modify base pose of the robot according to your environment. Therefor you can use `rtde_r.getActualQ()` function to obtain joint angles in your base pose and paste them in `base()` function instead of default angles.

8. You can modify control settings just at the beginning of `teleop_lib.py` file. In function `isInJointLimits` you can modify joint limits of your robot. We disabled check of this limits for now to avoid possible problems at the first launch, but you can easily enable it in `teleop_ur.py file`.

### Usage

1. Launch SteamVR, make sure that headset and tracker/controller are activ.

2. Run Unity project, make sure that you see camera stream.

3. Run `teleop_ur.py` file, pay attention to possible error messages that will be printed in console.

4. If all starts successfully, robot will follow your tracker.

5. To stop teleoperation, press `s` button. You will see all other instructions in your console.

## Acknowledgments

* [Unity project for multy-webcam streaming into VR Headset](https://github.com/hiro-wpi/VIVE-Motion-Capture-With-Wearable-Cameras?ysclid=m4h251pnea588029740)
* [ROS1 project for communication with EPOS controllers](https://github.com/jstiefel/MaxonEPOS2_ROS)