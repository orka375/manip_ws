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


    relay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('youbot'),
                'launch',
                'launch_relay.launch.py'
            )
        )
    )

    heartbeater_node = Node(
            package='youbot',
            executable='heart_beat_node',
            name='heart_beat_node',
            output='both',
        )


    simrealselector_launch = Node(
            package='youbot',
            executable='simreal_selector',
            name='simreal_selector',
            output='both',
        )
    
    opcua_node = Node(
            package='youbot',
            executable='opcua_client_node',
            name='opcua_client_node',
            output='both',
        )

    ro_bridge_node = Node(
            package='youbot',
            executable='ro_bridge',
            name='ro_bridge',
            output='both',
        )
    
    sm_node = Node(
            package='youbot',
            executable='statemachine',
            name='statemachine',
            output='both',
        )
    



    return LaunchDescription([
        heartbeater_node,
        simrealselector_launch,
        opcua_node,
        ro_bridge_node,
        relay_launch,
        sm_node,
    ])
