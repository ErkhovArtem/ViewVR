import keyboard
from time import sleep

def handle_user_input(robot):
    try: 
        if keyboard.is_pressed('s'):
            print('---------------------------------')
            print('Program stopped. Possible options:')
            print('e - exit program')
            print('b - move to base pose')
            print('r - run from current pose')
            return pause(robot)
    except Exception as e:
        print(f"Keyboard input error: {e}")
    
    return 0

def pause(robot):
    """Pause program execution and handle user input commands.
    
    Returns:
        int: -1 to exit program, 0 to continue execution
    """
    timeout = False
    while(True):
        try: 
            if keyboard.is_pressed('e'):
                return -1

            if keyboard.is_pressed('b'):
                print('---------------------------------')
                print('Move to base pose...')
                print('Program stopped. Possible options:')
                print('e - exit program')
                print('b - move to base pose')
                print('r - run from current pose')
                robot.move_to_base_pose()
                timeout = True

            if keyboard.is_pressed('r'):
                print('---------------------------------')
                print('Program running...')
                return 1
        except Exception as e:
            print(f"Keyboard input error: {e}")

        if timeout:
            # small timeout to prevent multiple triggering
            timeout = False
            sleep(0.1)