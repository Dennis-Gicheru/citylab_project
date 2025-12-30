import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('robot_patrol')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'patrol_config.rviz')
    
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_node',
            output='screen',
            remappings=[
        ('/cmd_vel', '/fastbot_1/cmd_vel'), ('/scan', '/fastbot_1/scan')
            ]
        ),

       Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', rviz_config_path],
           output='screen'
       )
    ])