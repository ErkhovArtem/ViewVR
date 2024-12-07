from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maxon_driver',
            # node_namespace='turtlesim1',
            executable='maxon_driver',
            # node_name='sim'
        ),
        Node(
            package='tcp_endpoint',
            # node_namespace='turtlesim2',
            executable='udp_server',
            # node_name='sim'
        ),
        ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "/epos_homing_service_roll ",
            "std_srvs/srv/Trigger ",
        ]],
        shell=True
        ),
        TimerAction(
            period=3.0,
            actions=[ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "/epos_homing_service_pitch ",
            "std_srvs/srv/Trigger ",
        ]],
        shell=True
        )],
        )
])
