from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_node',
            output='screen',
            remappings=[
        ('/cmd_vel', '/fastbot_1/cmd_vel'), ('/scan', '/fastbot_1/scan')
            ]
        ),
    ])