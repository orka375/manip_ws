import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'youbot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    bringup_dir = get_package_share_directory('youbot')

    # -------- Launch Arguments --------
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    default_world = os.path.join(bringup_dir, 'worlds', 'obstacles.world')

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world file'
    )

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch RViz GUI'
    )

    declare_gz_args = DeclareLaunchArgument(
    'gz_args',
    default_value=['-r -v2 ', LaunchConfiguration('world')],
    description='Arguments for gz_sim'
    )  

    # -------- Robot Description --------
    default_value=os.path.join(bringup_dir,'description','robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', default_value, ' sim_mode:=', use_sim_time]),value_type=str)

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}],
    )

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time]),
        value_type=str
    )

    # Define controller parameters file path
    controllers_config_file = os.path.join(pkg_share, 'config', 'controllers.yaml')


    # -------- Gazebo & Entity Spawn --------
     

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v2 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')


    # -------- Controllers --------
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--ros-args', '--params-file', controllers_config_file],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}] # Ensure controller manager also uses sim time
    # )

    # mecanum_drive_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['mec_dr_cont', '--ros-args', '--params-file', controllers_config_file],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}] # Ensure controller manager also uses sim time
    # )

    mec_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mec_dr_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # -------- MANIP -------------------
    arm_gui_config = os.path.join(pkg_share, 'config', 'arm_joints.yaml')

    # manip_joint_state_pub_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[arm_gui_config],
    #     condition=IfCondition(gui),
    #     output='screen',
    # )

    # -------- ROS-Gazebo Bridge --------
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # -------- RViz --------
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(gui)
    )

    # delay_rviz_after_joint_state = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node]
    #     )
    # )

    #-----------Joystick-----------------
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )

    return LaunchDescription([
        declare_world_arg,
        ros_gz_bridge,
        declare_sim_time_arg,
        declare_gui_arg,
        declare_gz_args,
        node_robot_state_publisher,
        world_arg,
        gazebo,
        joystick,
        spawn_entity,
        mec_drive_spawner,
        joint_broad_spawner,
        rviz_node
        # joint_state_broadcaster_spawner,
        # manip_joint_state_pub_gui_node,

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[mecanum_drive_spawner]
        #     )
        # ),

        
        # delay_rviz_after_joint_state
    ])