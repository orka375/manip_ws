import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    package_name = 'manip'
    bringup_dir = get_package_share_directory(package_name)

    # ----- Arguments -----
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # ----- Robot State Publisher -----
    default_value = os.path.join(bringup_dir, 'description', 'robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', default_value, ' sim_mode:=', use_sim_time]),value_type=str)


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}],
    )

    # ----- Controllers -----
    robot_description = {'robot_description': robot_description_config}
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'controllers.yaml',
    ])

        # Controller Manager (ros2_control_node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    # Spawners (but do not add them directly to LaunchDescription)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    )

    # Event Handlers to ensure controllers spawn after ros2_control_node starts
    delayed_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delayed_joint_broad = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[joint_broad_spawner],
        )
    )
        
    # joint_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    # )

    # ----- Visualization (RViz) -----
    # rviz_config_file = os.path.join(bringup_dir, 'config', 'rviz_config.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    #     condition=IfCondition(gui),
    # )

    # ----- Launch Description -----
    return LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
    DeclareLaunchArgument('gui', default_value='true', description='Start RViz2 automatically with this launch file.'),
    control_node,
    node_robot_state_publisher,
    delayed_joint_state_broadcaster,
    delayed_joint_broad,
    # rviz_node,
])
