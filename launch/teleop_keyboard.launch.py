from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node: teleop_twist_keyboard (publishes /cmd_vel)
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
    )

    # Node: twist_stamper (takes /cmd_vel_joy and outputs TwistStamped)
    twist_stamper_keyboard = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ]
    )

    return LaunchDescription([
        teleop_keyboard_node,
        twist_stamper_keyboard
    ])
