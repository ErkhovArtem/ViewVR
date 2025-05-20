### Robot Teleoperation system based on HTC VIVE PRO and UR3 manipulator

Robot teleoperation system based on `UR3` manipulator and `HTC VIVE PRO`, easily adaptable to your setup.

Code related to the paper **ViewVR: Visual-Feedback-Modes-to-Achieve-Quality-of-VR-based-Telemanipulation** was moved to separate branch.

### Hardware requirements

We use `UR3` robot and `Robotiq 2F-85` gripper with custom gripper control device.

`HTC VIVE PRO VR` MoCap system used to track operator's hand position.

### Installation

1. Setup environment:

````sh
git clone https://github.com/ErkhovArtem/ViewVR.git
cd ViewVR
pip install -r requirements.txt
cd src
````

2. If you use VIVE MoCap system, Generate `config.json` file using folowing instructions: https://pypi.org/project/vive-tracker-apiserver/

3. Provide configuration parameters of your setup in `config.py` file.

4. Modify robot's and device (tracker) interfaces according to your hardware / needs.


### Usage

1. Launch SteamVR, make sure that headset and tracker/controller are activ.

2.

````sh
sudo python main.py
````

3. If all starts successfully, robot will follow your tracker. To stop teleoperation, press `s` button. You will see all other instructions in terminal.