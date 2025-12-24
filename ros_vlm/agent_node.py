"""
ROS2 VLM Agent Node

A generic ROS2 agent that can introspect and interact with the ROS2 system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

from langchain.agents import Tool, AgentExecutor, create_react_agent
from langchain_core.prompts import PromptTemplate
from langchain.memory import ConversationBufferMemory

from ros_vlm.tools import (
    list_topics,
    list_nodes,
    list_services,
    echo_topic,
    publish_message
)


# TODO 6: Make this configurable via environment variable
# Support: 'together', 'groq', 'openai', 'ollama'
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
            model=os.getenv("LLM_MODEL", "meta-llama/Llama-3.3-70B-Instruct-Turbo-Free"),
            api_key=os.getenv("GROQ_API_KEY"),
            max_tokens=512,
            temperature=0.7,
        )
    elif provider == "openai":
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(
            model=os.getenv("LLM_MODEL", "gpt-3.5-turbo"),
            temperature=0.7,
            max_tokens=512,
            api_key=os.getenv("OPENAI_API_KEY"),
            
        )
    
    else:
        raise ValueError(f"Unknown LLM provider: {provider}")


SYSTEM_PROMPT = """You are a helpful ROS2 assistant. You can interact with the ROS2 system.

Available tools:
{tools}

Tool names: {tool_names}

You have access to the following capabilities:
1. List all active topics, nodes, and services
2. Read messages from any topic
3. Publish messages to any topic

When the user asks about the robot or ROS2 system, use the appropriate tools.
Be concise and helpful.

Previous conversation:
{chat_history}

User: {input}
{agent_scratchpad}"""


class AgentNode(Node):
    def __init__(self):
        super().__init__("ros2_agent")
        
        self.llm = get_llm()
        self.memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)
        
        self.tools = [
            Tool(name="list_topics", func=list_topics, description="List all ROS2 topics"),
            Tool(name="list_nodes", func=list_nodes, description="List all active ROS2 nodes"),
            Tool(name="list_services", func=list_services, description="List all ROS2 services"),
            Tool(name="echo_topic", func=echo_topic, description="Read latest message from a topic"),
            Tool(name="publish_message", func=publish_message, description="Publish a message to a topic"),
        ]
        
        prompt = PromptTemplate.from_template(SYSTEM_PROMPT)
        agent = create_react_agent(self.llm, self.tools, prompt)
        self.agent_executor = AgentExecutor(
            agent=agent,
            tools=self.tools,
            memory=self.memory,
            verbose=True,
            handle_parsing_errors=True
        )
        
        self.subscription = self.create_subscription(
            String, "agent_input", self.input_callback, 10
        )
        self.publisher = self.create_publisher(String, "agent_response", 10)
        
        self.get_logger().info("ROS2 Agent ready! Send messages to /agent_input")

    def input_callback(self, msg):
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
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
