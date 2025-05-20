import numpy as np

# robot base pose expressed in 6 joint angles
robot_base_pose = np.array([-0.09973652, 
          -1.6148599 ,  
          1.5400181 ,  
          0.1607126 ,  
          1.4716442 ,
        3.1881762 ])

# joint limits in following format (optional parameter):
# joint_limits = dict(
#     joint_name = (lower limit, upper limit), joint_name = ...)
joint_limits = None

# confuguration parameters of the robot
robot_config = dict(
ip="192.168.1.110",
base_pose= robot_base_pose,
use_gripper=True,
joint_limits = joint_limits)

# confuguration parameters of teleoperation device (and gripper controller, if provided)
device_config = dict(
tracker = "tracker_0",
serial_port = "COM3")

