# Robotic head driver

This ROS2 workspace provides code to control Maxon motors on 2-DoF robotic head using EPOS2 positional controllers. It based on [link](https://github.com/jstiefel/MaxonEPOS2_ROS) but several changes are made in accordance with our task:
* Migrated to ROS2.
* Added code to connect motors via CAN bus as Slaves to the Master one, that is connected via USB to PC.
* Added package for UDP/TCP connection.
* Minor changes in Homing mode.
* Final position can be set, that will be executed after node is stopped, for safety.
