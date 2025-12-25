"""
ROS2 MCP Agent Node

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import asyncio
import json

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from langchain.agents import AgentExecutor, create_react_agent
from langchain_core.prompts import PromptTemplate
from langchain_core.tools import StructuredTool
from langchain.memory import ConversationBufferMemory


def get_llm():
    """Get the LLM based on environment configuration."""
    provider = os.getenv("LLM_PROVIDER", "together").lower()
    
    if provider == "together":
        from langchain_together import ChatTogether
        return ChatTogether(
            model=os.getenv("LLM_MODEL", "meta-llama/Llama-3.3-70B-Instruct-Turbo-Free"),
            api_key=os.getenv("TOGETHER_API_KEY"),
            max_tokens=512,
            temperature=0.7,
        )
    elif provider == "groq":
        from langchain_groq import ChatGroq
        return ChatGroq(
            model=os.getenv("LLM_MODEL", "llama-3.1-70b-versatile"),
            api_key=os.getenv("GROQ_API_KEY"),
            max_tokens=512,
            temperature=0.7,
        )
    elif provider == "openai":
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(
            model=os.getenv("LLM_MODEL", "gpt-4o-mini"),
            temperature=0.7,
            max_tokens=512,
            api_key=os.getenv("OPENAI_API_KEY"),
        )
    else:
        raise ValueError(f"Unknown LLM provider: {provider}")


SYSTEM_PROMPT = """You are a helpful ROS2 assistant connected via MCP protocol.

Available tools:
{tools}

Tool names: {tool_names}

You can:
1. List topics, nodes, and services
2. Read messages from topics
3. Publish messages to topics
4. Call services
5. Inspect message types

Be concise and use tools when needed.

{chat_history}

User: {input}
{agent_scratchpad}"""


class MCPToolWrapper:
    """Wraps MCP tools for use with LangChain."""
    
    def __init__(self, session: ClientSession, tool_name: str):
        self.session = session
        self.tool_name = tool_name
    
    async def call_async(self, **kwargs) -> str:
        result = await self.session.call_tool(self.tool_name, kwargs)
        if result.content:
            return result.content[0].text
        return "No response"
    
    def __call__(self, **kwargs) -> str:
        loop = asyncio.get_event_loop()
        return loop.run_until_complete(self.call_async(**kwargs))


class MCPAgentNode(Node):
    def __init__(self):
        super().__init__("mcp_agent")
        
        self.llm = get_llm()
        self.memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)
        self.mcp_session = None
        self.tools = []
        self.agent_executor = None
        
        self.subscription = self.create_subscription(
            String, "agent_input", self.input_callback, 10
        )
        self.publisher = self.create_publisher(String, "agent_response", 10)
        
        # Start MCP connection in background
        self.create_timer(1.0, self.setup_mcp_once)
        self._mcp_ready = False
        
        self.get_logger().info("MCP Agent starting... waiting for MCP server connection")

    def setup_mcp_once(self):
        """One-time setup of MCP connection."""
        if self._mcp_ready:
            return
        
        try:
            asyncio.get_event_loop().run_until_complete(self.connect_to_mcp())
            self._mcp_ready = True
            self.get_logger().info("MCP Agent ready! Send messages to /agent_input")
        except Exception as e:
            self.get_logger().error(f"MCP connection failed: {e}")

    async def connect_to_mcp(self):
        
        server_script = os.path.expanduser("~/rs_ws/src/ros-vlm/ros_vlm/mcp_server.py")
        
        server_params = StdioServerParameters(
            command="python3",
            args=[server_script]
        )
        
        # Connect to MCP server
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                self.mcp_session = session
                
                # Get available tools from server
                tools_response = await session.list_tools()
                
                # Convert MCP tools to LangChain tools
                for mcp_tool in tools_response.tools:
                    wrapper = MCPToolWrapper(session, mcp_tool.name)
                    
                    # Build input schema for StructuredTool
                    lc_tool = StructuredTool.from_function(
                        func=wrapper,
                        name=mcp_tool.name,
                        description=mcp_tool.description,
                    )
                    self.tools.append(lc_tool)
                
                # Create agent
                prompt = PromptTemplate.from_template(SYSTEM_PROMPT)
                agent = create_react_agent(self.llm, self.tools, prompt)
                self.agent_executor = AgentExecutor(
                    agent=agent,
                    tools=self.tools,
                    memory=self.memory,
                    verbose=True,
                    handle_parsing_errors=True
                )

    def input_callback(self, msg):
        if not self._mcp_ready or not self.agent_executor:
            self.get_logger().warn("Agent not ready yet, please wait...")
            return
        
        self.get_logger().info(f"Received: {msg.data}")
        
        try:
            result = self.agent_executor.invoke({"input": msg.data})
            response = result.get("output", "No response")
        except Exception as e:
            self.get_logger().error(f"Agent error: {e}")
            response = f"Error: {e}"
        
        self.get_logger().info(f"Response: {response}")
        
        response_msg = String()
        response_msg.data = response
        self.publisher.publish(response_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MCPAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
