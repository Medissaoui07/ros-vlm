# ROS2 MCP Agent

A ROS2 agent that uses **Model Context Protocol (MCP)** for modular tool management.

## Architecture

```
┌────────────────┐      ┌────────────────┐      ┌────────────────┐
│   MCP Agent    │─────▶│   MCP Server   │─────▶│     ROS2       │
│  (LLM Client)  │◀─────│  (ROS2 Tools)  │◀─────│    System      │
└────────────────┘      └────────────────┘      └────────────────┘
```

## Features

- **MCP Protocol**: Standard interface for LLM-tool communication
- **ROS2 Introspection**: List topics, nodes, services
- **Topic I/O**: Read and publish to any topic
- **Service Calls**: Call any ROS2 service
- **Modular**: Add more MCP servers for new capabilities

## Quick Start

```bash
# Install dependencies
pip install mcp langchain langchain-together

# Set API key
export TOGETHER_API_KEY="your-key"

# Build
cd ~/rs_ws
colcon build --packages-select ros_vlm
source install/setup.bash

# Run demo
ros2 launch ros_vlm mcp_demo.launch.py

# Send command
ros2 topic pub /agent_input std_msgs/msg/String "data: 'list topics'" --once
```

## Project Structure

```
ros_vlm/
├── mcp_agent_node.py   # MCP client agent
├── mcp_server.py       # ROS2 tools server
└── tools/
    └── ros2_tools.py   # Tool implementations
```

## LLM Providers

Set `LLM_PROVIDER` env var:
- `together` (default)
- `groq`
- `openai`
