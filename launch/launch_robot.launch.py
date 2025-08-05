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



    package_name='manip'
    bringup_dir = get_package_share_directory('manip')


# ----- Arguments -----

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    



   

# ----- Robot State Publisher -----


    default_value=os.path.join(bringup_dir,'description','robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', default_value, ' sim_mode:=', use_sim_time]),value_type=str)

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}],
    )




# ----- Joint State Broadcaster -----

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )



# ----- Controllers -----

    robot_description = {"robot_description": robot_description_config}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("youbot"),
            "config",
            "controllers.yaml",
        ]
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    robot_controllers],
        output="both",
    )



    meca_drive_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=["pos_contr", "--controller-manager", "/controller_manager"],
            
    )


    delayed_meca_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[meca_drive_spawner],
        )
    )





# ----- Visualization of Robot RVIZ -----


    rviz_config_file = os.path.join(bringup_dir, 'config', 'rviz_config.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )





# ----- Launch -----

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
           "gui",
           default_value="true",
           description="Start RViz2 automatically with this launch file.",
        ),



        # checkPowerService,
        # delayed_enablePower_handler,
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        delayed_meca_drive_spawner,

    ])
