import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    generation_node = Node(
        package= 'minichallenge5',
        executable= 'line_follower',
        output = 'screen',
    )

    traffic_node = Node(
        package= 'minichallenge5',
        executable= 'traffic_lights',
        output = 'screen',
    )

    ld = LaunchDescription([generation_node, traffic_node])
    return ld