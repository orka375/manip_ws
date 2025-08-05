import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
from launch.actions import ExecuteProcess, LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():



    package_name='youbot'



    
    relaycontrol = Node(
            package='youbot',
            executable='relay_control_node',
            name='relay_control_node',
            output='both',
        )
    


    setR1false = ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/set_PowerBase',
                        'std_srvs/srv/SetBool',
                        '{data: false}'
                    ],
                    output='screen'
                )
    
    setR2false = ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/set_PowerArm',
                        'std_srvs/srv/SetBool',
                        '{data: false}'
                    ],
                    output='screen'
                )
    
    setR3false = ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/set_Res1',
                        'std_srvs/srv/SetBool',
                        '{data: false}'
                    ],
                    output='screen'
                )
    
    setR4false = ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/set_Res2',
                        'std_srvs/srv/SetBool',
                        '{data: false}'
                    ],
                    output='screen'
                )

    setrelayfalse = TimerAction(
                    period=1.0,
                    actions=[setR1false,
                                setR2false,
                                setR3false,
                                setR4false,]
                        
                    
                )

    # Launch them all!
    return LaunchDescription([

        relaycontrol,
        RegisterEventHandler(
            OnProcessStart(
                target_action=relaycontrol,
                on_start=[
                    LogInfo(msg='✅ relay_control_node has started.'),
                    setrelayfalse
                ]
            )
        ),


        
    ])