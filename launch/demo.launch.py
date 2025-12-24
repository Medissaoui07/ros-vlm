"""Demo launch file - runs agent with turtlesim for testing."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Turtlesim node for testing
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    # The ROS2 VLM agent
    agent = Node(
        package='ros_vlm',
        executable='agent_node',
        name='ros2_agent',
        output='screen'
    )
    
    return LaunchDescription([turtlesim, agent])
