"""
ROS2 Introspection and Interaction Tools

These tools allow the agent to discover and interact with the ROS2 system.
"""

from langchain.tools import tool
import subprocess
import json



@tool
def list_topics(filter_pattern: str = "") -> str:
    """
    List all available ROS2 topics.
    Args:
        filter_pattern: Optional pattern to filter topics (e.g., 'cmd' to find cmd_vel)
    Returns:
        List of topic names
    """
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        topics = result.stdout.strip().split('\n')
        
        if filter_pattern:
            topics = [t for t in topics if filter_pattern.lower() in t.lower()]
        
        if not topics or topics == ['']:
            return "No topics found"
        
        return f"Available topics ({len(topics)}):\n" + "\n".join(topics)
    except Exception as e:
        return f"Error listing topics: {e}"


# TODO 2: Implement list_nodes
# Hint: Use subprocess to run 'ros2 node list' command
@tool
def list_nodes(filter_pattern: str = "") -> str:
    """
    List all active ROS2 nodes.
    Args:
        filter_pattern: Optional pattern to filter nodes
    Returns:
        List of node names
    """

    try : 
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        nodes = result.stdout.strip().split('\n')

        if filter_pattern:
            nodes = [n for n in nodes if filter_pattern.lower() in n.lower()]

        if not nodes or nodes == ['']:
            return "No nodes found"

        return f"Active nodes ({len(nodes)}):\n" + "\n".join(nodes)
    except Exception as e : 
        return f"Error listing nodes: {e}"

   



@tool
def list_services(filter_pattern: str = "") -> str:
    """
    List all available ROS2 services.
    Args:
        filter_pattern: Optional pattern to filter services
    Returns:
        List of service names
    """
    try:
        result = subprocess.run(
            ['ros2', 'service', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        services = result.stdout.strip().split('\n')

        if filter_pattern:
            services = [s for s in services if filter_pattern.lower() in s.lower()]

        if not services or services == ['']:
            return "No services found"

        return f"Available services ({len(services)}):\n" + "\n".join(services)
    except Exception as e:
        return f"Error listing services: {e}"


# TODO 4: Implement echo_topic
# Hint: Use 'ros2 topic echo <topic> --once' to get one message
@tool
def echo_topic(topic_name: str) -> str:
    """
    Get the latest message from a ROS2 topic.
    Args:
        topic_name: The topic to read from (e.g., '/cmd_vel')
    Returns:
        The message content as string
    """
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', topic_name, '--once'],
            capture_output=True,
            text=True,
            timeout=3
        )
        return result.stdout.strip()
    except Exception as e:
        return f"Error echoing topic {topic_name}: {e}"



@tool  
def publish_message(topic_name: str, message_type: str, message_data: str) -> str:
    """
    Publish a message to a ROS2 topic.
    Args:
        topic_name: Target topic (e.g., '/cmd_vel')
        message_type: Message type (e.g., 'geometry_msgs/msg/Twist')
        message_data: YAML-formatted message data
    Returns:
        Confirmation or error message
    
    Example:
        publish_message('/cmd_vel', 'geometry_msgs/msg/Twist', 
                       '{linear: {x: 0.5}, angular: {z: 0.0}}')
    """
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'pub', topic_name, message_type, message_data, '--once'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return f"Published message to {topic_name}: {result.stdout.strip()}"
    except Exception as e:
        return f"Error publishing message to {topic_name}: {e}"


# BONUS TODO: Get topic info (message type, publishers, subscribers)
@tool
def get_topic_info(topic_name: str) -> str:
    """
    Get information about a ROS2 topic.
    Args:
        topic_name: The topic to get info about
    Returns:
        Topic type, publishers, and subscribers
    """
    # Hint: Use 'ros2 topic info <topic> -v'
    try : 
        result = subprocess.run(
        ['ros2', 'topic', 'info', topic_name, '-v'],
        capture_output=True,
        text=True,
        timeout=5
    )
        return result.stdout.strip()
    except Exception as e :
        return f"Error getting topic info for {topic_name}: {e}"
    