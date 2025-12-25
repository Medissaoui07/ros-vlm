#!/usr/bin/env python3
"""
ROS2 MCP Server

Exposes ROS2 introspection and interaction tools via MCP protocol.
Can be used by any MCP client (Claude Desktop, our agent, etc.)

Run with: python -m ros_vlm.mcp_server
"""

import subprocess
import asyncio
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool, TextContent


app = Server("ros2-mcp-server")


# Helper function to run ROS2 commands
def run_ros2_command(args: list, timeout: int = 5) -> str:
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.stdout.strip() or result.stderr.strip()
    except subprocess.TimeoutExpired:
        return f"Command timed out after {timeout}s"
    except Exception as e:
        return f"Error: {e}"


@app.list_tools()
async def list_tools() -> list[Tool]:
    """Define all available ROS2 tools."""
    return [
        Tool(
            name="list_topics",
            description="List all available ROS2 topics",
            inputSchema={
                "type": "object",
                "properties": {
                    "filter": {"type": "string", "description": "Optional filter pattern"}
                }
            }
        ),
        Tool(
            name="list_nodes",
            description="List all active ROS2 nodes",
            inputSchema={
                "type": "object",
                "properties": {
                    "filter": {"type": "string", "description": "Optional filter pattern"}
                }
            }
        ),
        Tool(
            name="list_services",
            description="List all available ROS2 services",
            inputSchema={
                "type": "object",
                "properties": {
                    "filter": {"type": "string", "description": "Optional filter pattern"}
                }
            }
        ),
        Tool(
            name="echo_topic",
            description="Get the latest message from a ROS2 topic",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "Topic name (e.g., /cmd_vel)"}
                },
                "required": ["topic"]
            }
        ),
        Tool(
            name="publish_message",
            description="Publish a message to a ROS2 topic",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "Topic name"},
                    "msg_type": {"type": "string", "description": "Message type (e.g., geometry_msgs/msg/Twist)"},
                    "data": {"type": "string", "description": "YAML message data"}
                },
                "required": ["topic", "msg_type", "data"]
            }
        ),
        Tool(
            name="get_topic_info",
            description="Get detailed information about a topic",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "Topic name"}
                },
                "required": ["topic"]
            }
        ),
        Tool(
            name="call_service",
            description="Call a ROS2 service",
            inputSchema={
                "type": "object",
                "properties": {
                    "service": {"type": "string", "description": "Service name"},
                    "srv_type": {"type": "string", "description": "Service type"},
                    "request": {"type": "string", "description": "YAML request data", "default": "{}"}
                },
                "required": ["service", "srv_type"]
            }
        ),
        Tool(
            name="describe_message_type",
            description="Show the structure of a message or service type",
            inputSchema={
                "type": "object",
                "properties": {
                    "msg_type": {"type": "string", "description": "Type (e.g., geometry_msgs/msg/Twist)"}
                },
                "required": ["msg_type"]
            }
        ),
    ]


@app.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    """Handle tool calls."""
    
    if name == "list_topics":
        output = run_ros2_command(['ros2', 'topic', 'list'])
        if arguments.get("filter"):
            lines = output.split('\n')
            lines = [l for l in lines if arguments["filter"].lower() in l.lower()]
            output = '\n'.join(lines)
    
    elif name == "list_nodes":
        output = run_ros2_command(['ros2', 'node', 'list'])
        if arguments.get("filter"):
            lines = output.split('\n')
            lines = [l for l in lines if arguments["filter"].lower() in l.lower()]
            output = '\n'.join(lines)
    
    elif name == "list_services":
        output = run_ros2_command(['ros2', 'service', 'list'])
        if arguments.get("filter"):
            lines = output.split('\n')
            lines = [l for l in lines if arguments["filter"].lower() in l.lower()]
            output = '\n'.join(lines)
    
    elif name == "echo_topic":
        topic = arguments["topic"]
        output = run_ros2_command(['ros2', 'topic', 'echo', topic, '--once'], timeout=3)
    
    elif name == "publish_message":
        topic = arguments["topic"]
        msg_type = arguments["msg_type"]
        data = arguments["data"]
        output = run_ros2_command(['ros2', 'topic', 'pub', topic, msg_type, data, '--once'])
    
    elif name == "get_topic_info":
        topic = arguments["topic"]
        output = run_ros2_command(['ros2', 'topic', 'info', topic, '-v'])
    
    elif name == "call_service":
        service = arguments["service"]
        srv_type = arguments["srv_type"]
        request = arguments.get("request", "{}")
        output = run_ros2_command(['ros2', 'service', 'call', service, srv_type, request], timeout=10)
    
    elif name == "describe_message_type":
        msg_type = arguments["msg_type"]
        output = run_ros2_command(['ros2', 'interface', 'show', msg_type])
    
    else:
        output = f"Unknown tool: {name}"
    
    return [TextContent(type="text", text=output)]


async def main():
    async with stdio_server() as (read_stream, write_stream):
        await app.run(read_stream, write_stream, app.create_initialization_options())


if __name__ == "__main__":
    asyncio.run(main())
