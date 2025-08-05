import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    bringup_dir = get_package_share_directory('manip')
    launch_dir = os.path.join(bringup_dir, 'launch')

    

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    urdf_file= LaunchConfiguration('urdf_file')
    xacro_file= LaunchConfiguration('xacro_file')
    # Process the URDF file

   
    
    declare_xacro = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(bringup_dir,'description','robot.urdf.xacro')
    )
    
    declare_urdf = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(bringup_dir, 'description', 'manip.xacro')
    )

    robot_description_config = ParameterValue(Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]),value_type=str)
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[
            ("/manip/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )


 


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),

        declare_urdf,
        declare_xacro,
        node_robot_state_publisher

    ])


