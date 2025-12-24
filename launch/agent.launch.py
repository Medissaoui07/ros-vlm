"""Agent-only launch file."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    agent = Node(
        package='ros_vlm',
        executable='agent_node',
        name='ros2_agent',
        output='screen'
    )
    
    return LaunchDescription([agent])
