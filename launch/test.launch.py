import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

import xacro



    


def generate_launch_description():



    package_name='youbot'
    bringup_dir = get_package_share_directory('youbot')

 
# ----- LCD Display -----   
    lcd_node = Node(
        package="youbot",
        executable="lcddisplay",
    )

    WelcDisp = TimerAction(
        period = 0.1,
        actions=[
            ExecuteProcess(
                cmd=['ros2 ','topic ','pub ','/lcd_line1 ','std_msgs/msg/String ','"{data: Welcome}" ','--once'],
                shell = True
            ),

        ]
    )
    

    # launch_InitDisp = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=WelcDisp,
    #         on_exit=[lcd_node]
    #     )
    # )



    delayed_IpDisp = TimerAction(
        period=6.0,  # delay in seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/lcd_show_ip',
                    'std_srvs/srv/Trigger'
                    ],
                output='screen'
            ),
        ]
    )





# ----- Launch -----

    return LaunchDescription([
        lcd_node,
        # launch_InitDisp,
        delayed_IpDisp,
        WelcDisp
       
    ])
