import numpy as np
from scipy.spatial.transform import Rotation as R

def provide_teleoperation(robot, device, device_pose, device_pose_init, tcp_pose_init):
    """
    Performs one step of teleoperation using robot's inverse kinematics and checks safety conditions, if defined.
    """
    robot_tcp_pose = retarget(device_pose, device_pose_init, tcp_pose_init)
    if robot.isPoseWithinSafetyLimits(robot_tcp_pose):
        robot_joint_angles = robot.getInverseKinematics(robot_tcp_pose)
        if robot.isPoseWithinJointLimits(robot_joint_angles):
            robot.move_to_pose(robot_joint_angles, device.get_gripper_state())

def retarget(device_pose, device_pose_init, tcp_pose_init, sens = 1.0):
    """
    Calculates target tcp pose by retargeting displacement between current and initail hand poses into robot's frame.
    Args:
        device_pose: current traker position - list of 7 values (x, y, z , qx, qy, qz, qw).
        device_pose_init: initial traker position - list of 7 values (x, y, z , qx, qy, qz, qw).
        tcp_pose_init: initial robot position - list of 6 values (x, y, z , rx, ry, rz).
    Returns:
        np.array: target tcp position.
    """
    tcp_pose = np.zeros(6)

    # position
    tcp_pose[0] = tcp_pose_init[0] + (device_pose[2] - device_pose_init[2])/sens
    tcp_pose[1] = tcp_pose_init[1] + (device_pose[0] - device_pose_init[0])/sens
    tcp_pose[2] = tcp_pose_init[2] + (device_pose[1] - device_pose_init[1])/sens
    
    # orientation
    rot_rel = R.from_quat(device_pose_init[3:]) * R.from_quat(device_pose[3:]).inv()
    rot = R.from_rotvec(tcp_pose_init[3:]) * rot_rel

    tcp_pose[3:6] = rot.as_rotvec()

    return tcp_pose

