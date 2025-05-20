from teleoperation.teleoperation import provide_teleoperation
from device.teleop_device import ViveTracker
from robot.robot import UR3Teleop
from utils.utils import handle_user_input
from config import *

def main():
    # hardware initilazation
    device = ViveTracker(**device_config)
    robot = UR3Teleop(**robot_config)

    # start from the base pose
    robot.move_to_base_pose()
            
    tcp_pose_init = robot.get_current_tcp_pose()
    device_pose_init = device.get_device_pose()

    print('Teleoperation started.')

    while True:         
        # retrieve current hand position
        device_pose = device.get_device_pose()

        # perform one step of teleoperation
        provide_teleoperation(robot, device, device_pose, device_pose_init, tcp_pose_init)

        # handle user input command
        cmd = handle_user_input(robot)
        match cmd:
            case -1:
                # exit program 
                break
            case 0:
                # if no command received
                pass
            case 1:
                # set new base poses for robot and hand if teleoperation was paused
                tcp_pose_init = robot.get_current_tcp_pose()
                device_pose_init = device.get_device_pose()

    robot.close()
    device.close()

if __name__ == "__main__":
    main()
    


