
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction  # Added TimerAction
from std_srvs.srv import Trigger 
def generate_launch_description():
    service_activator = ExecuteProcess(
        cmd=['ros2 service call /activate_true_north std_srvs/srv/Empty'],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        Node(
            package='true_north_calculator',
            executable='true_north_service',
            name='true_north_service',
            output='screen'
        ),
        Node(
            package='true_north_calculator',
            executable='true_north_controller',
            name='true_north_controller',
            output='screen'
        ),
        Node(
            package='true_north_calculator',
            executable='true_north_manager',
            name='true_north_manager',
            output='screen'
        ),
        TimerAction(
            period=10.0,  #10 second delay
            actions=[ExecuteProcess(
                cmd=['ros2 service call /activate_controller std_srvs/srv/Trigger'],
                output='screen',
                shell=True
            )]
        )
    ])

