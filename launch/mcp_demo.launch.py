"""MCP Agent launch file with turtlesim demo."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    mcp_agent = Node(
        package='ros_vlm',
        executable='mcp_agent',
        name='mcp_agent',
        output='screen'
    )
    
    return LaunchDescription([turtlesim, mcp_agent])
