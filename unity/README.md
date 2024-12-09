# Webcam streaming into HTC VIVE Headset

This system consists of HTC VIVE VR headset, trackers and various cameras. It provides camera stream from various cameras into your HTC VIVE headset. This project is based on [link](https://github.com/hiro-wpi/VIVE-Motion-Capture-With-Wearable-Cameras?ysclid=m4h251pnea588029740)

This repository is tested in Unity 2020+, but other Unity version should work as well.

## Installation

  In Unity hub, start a new empty 3D project.

  The basic package you need is "[SteamVR Plugin](https://assetstore.unity.com/packages/tools/integration/steamvr-plugin-32647)". Please make sure you put it in your Unity asset, import and download it in this project. After downloading, navigate to `Window -> SteamVR Input` and follow the instruction to generate necessary files.

Git clone this repository. Merge the downloaded folder into your project folder, or put all the files and folders of this repository under your project folder. Make sure your <u>SteamVR</u> is launched and all the components are paired before running any scenes.

## Usage

- Open scene **WebCamStreaming**.
- Navigate to `Scripts` game object and make sure `Use Predefined Cameras` under the <u>Web Cam Stream</u> script is unticked. Hit play in Unity and all the camera names will be loaded in the `Device Names`. Copy down the names of the cameras you want to use, stop Unity, tick `Use Predefined Cameras` and enter these names in the `Device Names` now.
- In field `path` enter absolute path to the folder where camera images will be saved, it's like a prototype of dataset collection system. It will work only if python client is connected. 
- Hit play in Unity, and you should see a camera view on the canvas. To switch to the next camera, press the space bar.


