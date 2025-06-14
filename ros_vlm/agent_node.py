import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_together import ChatTogether
from langchain.memory import ConversationBufferMemory

# --- LangChain Core components ---
from langchain.agents import Tool # Still need Tool to wrap vision_tool
from langchain_core.agents import AgentAction, AgentFinish # For defining what the LLM outputs
from langchain_core.messages import BaseMessage, HumanMessage, FunctionMessage # For chat history and tool outputs
from langchain_core.prompts import ChatPromptTemplate # For structuring the LLM's prompt

# --- LangGraph components ---
from typing import List, Tuple, Union, TypedDict
from langgraph.graph import StateGraph, END # Core LangGraph components

# --- Your utility imports ---
from ros_vlm.utils.prompt_loader import PromptLoader
from ros_vlm.utils.tools import vision_tool, set_global_vision_context # Your custom vision tool

import os
import time

# --- Define the Graph State ---
# This defines what information is passed between nodes in your LangGraph.
class AgentState(TypedDict):
    input: str # The current user input or query
    chat_history: List[BaseMessage] # The accumulated conversation history
    intermediate_steps: List[Tuple[AgentAction, str]] # The intermediate steps (Action, Observation)
    agent_outcome: Union[AgentAction, AgentFinish, None] # The agent's decision

class AgentNode(Node):
    def __init__(self):
        super().__init__("agent_node")
        self.api_key = os.getenv("TOGETHER_API_KEY")

        # Ensure API key is set
        if not self.api_key:
            self.get_logger().error("TOGETHER_API_KEY environment variable not set.")
            raise EnvironmentError("TOGETHER_API_KEY environment variable not set.")

        self.prompt_loader = PromptLoader()

        # Rate limiting parameters for requests to the LLM/agent
        self.last_request_time = 0
        self.base_interval = 20  # Base interval between requests (e.g., 20 seconds)
        self.max_interval = 60   # Maximum interval for exponential backoff
        self.current_interval = self.base_interval
        self.request_queue = []  # Queue for pending requests

        # Initialize the Language Model (LLM)
        self.llm = ChatTogether(
            model="meta-llama/Llama-3.3-70B-Instruct-Turbo-Free",
            api_key=self.api_key,
            max_tokens=256,
            temperature=0.7,
        )

        # Initialize conversation memory for chat history
        self.memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True, # Ensure messages are returned as BaseMessage objects
        )

        # Define the tools available to the agent.
        # Currently, only the vision_tool is included.
        self.tools = [
            Tool(
                name="vision_tool",
                func=vision_tool,
                description="Use this tool to get the most recent visual information available from the robot's perception. This tool takes no direct input; it will fetch the latest vision data directly."
            ),
            # Add other tools here in the future if needed, e.g., robot control tools
        ]

        # --- LangGraph Setup ---
        # 1. Create your graph instance with the defined state
        workflow = StateGraph(AgentState)

        # 2. Add the main nodes (steps) to the graph
        workflow.add_node("agent", self._call_agent_node) # Node where the LLM reasons
        workflow.add_node("tool", self._call_tool_node)   # Node where tools are executed

        # 3. Define the starting point of the graph
        workflow.set_entry_point("agent")

        # 4. Define the edges (transitions between nodes)
        # Conditional edge from "agent" node:
        # Based on the LLM's output, decide whether to call a tool or end.
        workflow.add_conditional_edges(
            "agent",                  # Source node
            self._should_continue,    # Function to determine next node
            {"continue": "tool",      # If _should_continue returns "continue", go to "tool" node
             "end": END}              # If _should_continue returns "end", terminate the graph
        )
        # Direct edge from "tool" node:
        # After executing a tool, always go back to the "agent" node
        # so the LLM can process the tool's output and decide next steps.
        workflow.add_edge("tool", "agent")

        # 5. Compile the graph into a runnable object
        self.agent_graph = workflow.compile()
        # --- End LangGraph Setup ---

        # ROS 2 Timer for processing requests from the queue
        self.create_timer(1.0, self.process_requests)

        # ROS 2 Subscriptions
        self.txt_subscription = self.create_subscription(
            String,
            "vorker_response",  # Topic to receive text input (e.g., from a speech-to-text node)
            self.response_callback,
            10
        )

        self.vision_subscription = self.create_subscription(
            String,
            "vlm_response",     # Topic to receive vision context from a VLM node
            self.vision_callback,
            10
        )

        # ROS 2 Publisher
        self.publisher = self.create_publisher(
            String,
            "agent_response",   # Topic to publish the agent's text response
            10
        )
        # Note: self.robot_cmd_publisher is NOT included as per your request.

    def vision_callback(self, msg):
        """Callback for receiving vision context."""
        set_global_vision_context(msg.data)  # Update the global vision context
        self.get_logger().info(f"Updated vision Context: {msg.data[:50]}...") # Log first 50 chars for brevity

    def response_callback(self, msg):
        """Callback for receiving user text input (e.g., from speech-to-text)."""
        self.request_queue.append(msg.data)
        self.get_logger().info(f"Received user input to process: {msg.data}")

    def process_requests(self):
        """
        Timer callback to process requests from the queue, respecting rate limits.
        """
        if not self.request_queue:
            return

        current_time = time.time()
        if current_time - self.last_request_time < self.current_interval:
            # Not enough time has passed, wait
            return

        user_input = self.request_queue.pop(0) # Get the oldest request
        self._run_agent(user_input)

        self.last_request_time = current_time
        # Reset interval after successful processing. Exponential backoff logic is in _run_agent's exception.
        self.current_interval = self.base_interval

    # --- LangGraph Node Implementations ---
    def _call_agent_node(self, state: AgentState) -> dict:
        """
        LangGraph node: The LLM processes the current state (input, history, tool outputs)
        and decides to call a tool (AgentAction) or provide a final answer (AgentFinish).
        """
        self.get_logger().info("LangGraph: Agent (LLM) Node is thinking...")

        # Construct the prompt for the LLM
        # This prompt tells the LLM about its role, available tools, and current conversation state.
        tools_description = "\n".join([f"{tool.name}: {tool.description}" for tool in self.tools])
        
        # Build the message history for the LLM's context
        messages_for_llm = self.memory.buffer_as_messages # Start with full chat history from memory

        # Create system message with tools description and instructions
        system_message = f"""You are a helpful robot assistant. You can see through my camera and help me understand what's happening.
Available tools:
{tools_description}

Instructions:
1. Only use the vision tool if the user specifically asks about what you can see
2. After using the vision tool, you must provide a final response to the user
3. Don't use the same tool multiple times unless specifically asked
4. Respond directly to the user's questions without using tools if possible

Current conversation state:"""
        messages_for_llm.insert(0, HumanMessage(content=system_message))

        # Add any previous tool actions and observations (intermediate steps)
        for action, observation in state["intermediate_steps"]:
            messages_for_llm.append(HumanMessage(content=action.log))
            messages_for_llm.append(FunctionMessage(name=action.tool, content=observation))

        # Add the current user's input for this turn
        messages_for_llm.append(HumanMessage(content=state["input"]))

        try:
            # Invoke the LLM, binding it to the available tools.
            # 'bind_tools' helps the LLM output structured data for tool calls.
            llm_with_tools = self.llm.bind_tools(self.tools)
            ai_message = llm_with_tools.invoke(messages_for_llm)

            self.get_logger().info(f"LLM Agent Output: {ai_message}")
            
            # Handle tool calls from AIMessage
            if ai_message.additional_kwargs.get('tool_calls'):
                # Check if we've already used tools in this turn
                if state["intermediate_steps"]:
                    # If we've already used a tool, force a final answer
                    return {"agent_outcome": AgentFinish(
                        return_values={"output": f"Based on what I can see: {ai_message.content if ai_message.content else 'Let me help you with that. ' + state['intermediate_steps'][-1][1]}"},
                        log="Final Answer after tool use"
                    )}
                tool_call = ai_message.additional_kwargs['tool_calls'][0]  # Get first tool call
                return {"agent_outcome": AgentAction(
                    tool=tool_call['function']['name'],
                    tool_input=tool_call['function']['arguments'],
                    log=f"Decided to use {tool_call['function']['name']}"
                )}
            else:
                # If no tool calls, treat as final answer
                return {"agent_outcome": AgentFinish(
                    return_values={"output": ai_message.content if ai_message.content else "Hello! I'm your robot assistant. How can I help you today?"},
                    log="Final Answer"
                )}

        except Exception as e:
            self.get_logger().error(f"Error in _call_agent_node (LLM invocation): {e}")
            # If the LLM call fails, return an AgentFinish with an error message
            return {"agent_outcome": AgentFinish(
                return_values={"output": f"Error: LLM could not decide next step: {e}"},
                log=f"Error in LLM decision: {e}"
            )}

    def _call_tool_node(self, state: AgentState) -> dict:
        """
        LangGraph node: Executes the tool requested by the LLM.
        """
        self.get_logger().info("LangGraph: Tool Node is executing...")
        
        # Get the AgentAction (tool call) from the previous 'agent' node's output
        agent_outcome = state["agent_outcome"]
        
        # Ensure the outcome is indeed an AgentAction (meaning a tool was requested)
        if not isinstance(agent_outcome, AgentAction):
            self.get_logger().error(f"Expected AgentAction, but received {type(agent_outcome)}. Skipping tool execution.")
            # If not an AgentAction, something went wrong, return current state or an error observation
            return {"intermediate_steps": [(AgentAction(tool="error", tool_input="Invalid outcome", log=""), f"Error: Unexpected agent outcome type: {type(agent_outcome)}")]}

        try:
            # Find the specific tool requested by name
            selected_tool = next((t for t in self.tools if t.name == agent_outcome.tool), None)
            
            if selected_tool:
                # Execute the tool function with the provided input
                tool_output = selected_tool.func(agent_outcome.tool_input)
                self.get_logger().info(f"Tool '{agent_outcome.tool}' executed. Output: {tool_output}")
                
                # Update the state by adding this tool's action and its output
                return {"intermediate_steps": [(agent_outcome, tool_output)]}
            else:
                self.get_logger().error(f"Tool '{agent_outcome.tool}' not found.")
                # If tool not found, add an error observation to intermediate steps
                return {"intermediate_steps": [(agent_outcome, f"Error: Tool '{agent_outcome.tool}' not found.")]}

        except Exception as e:
            self.get_logger().error(f"Error executing tool '{agent_outcome.tool}': {e}")
            # If tool execution fails, add an error observation to intermediate steps
            return {"intermediate_steps": [(agent_outcome, f"Error executing tool '{agent_outcome.tool}': {e}")]}

    def _should_continue(self, state: AgentState) -> str:
        """
        LangGraph conditional edge: Determines the next step in the graph based on the agent's outcome.
        Returns "continue" to transition to the tool node, or "end" to finish the graph.
        """
        self.get_logger().info("LangGraph: Deciding next step in workflow...")
        
        # Get the outcome from the last 'agent' node
        agent_outcome = state["agent_outcome"]

        # If the LLM's outcome is a tool action, the graph should continue to the "tool" node
        if isinstance(agent_outcome, AgentAction):
            self.get_logger().info(f"Agent wants to use tool: {agent_outcome.tool}")
            return "continue"
        # If the LLM's outcome is a final answer, the graph should terminate
        elif isinstance(agent_outcome, AgentFinish):
            self.get_logger().info("Agent has a final answer.")
            return "end"
        else:
            # This case indicates an unexpected output from the LLM, force graph termination
            self.get_logger().error(f"Unexpected agent outcome type: {type(agent_outcome)}. Forcing graph end.")
            return "end"

    # --- Main Agent Run Logic ---
    def _run_agent(self, user_input: str):
        """
        Initiates and runs the LangGraph agent for a given user input.
        """
        self.get_logger().info(f"Running agent with input: {user_input}")
        
        try:
            # Initialize the graph's state with the current input and memory
            initial_state = {
                "input": user_input,
                "chat_history": self.memory.buffer_as_messages,
                "intermediate_steps": [] # Start with an empty list of intermediate steps for this turn
            }

            # Invoke (run) the compiled LangGraph.
            # Using .stream() allows you to see the state updates at each step, useful for debugging.
            final_state = {}
            for s in self.agent_graph.stream(initial_state):
                self.get_logger().info(f"LangGraph Stream Step: {s}") # Log each step's state
                final_state = s # Keep track of the last state reached in the graph

            # Extract the final response from the graph's final state
            agent_outcome = final_state.get("agent_outcome")
            
            if isinstance(agent_outcome, AgentFinish):
                agent_response = agent_outcome.return_values["output"]
            else:
                # This block would execute if the graph somehow ended without an AgentFinish
                agent_response = "Error: Agent did not provide a final answer for this query."
                self.get_logger().error(f"LangGraph finished without an AgentFinish outcome. Final state: {final_state}")
            
            # --- Manually update conversation memory ---
            # In a custom LangGraph, memory is not automatically updated by default.
            self.memory.chat_memory.add_user_message(user_input)
            self.memory.chat_memory.add_ai_message(agent_response)

            self.get_logger().info(f"Agent final response: {agent_response}")

            # Publish the agent's final text response
            response_msg = String()
            response_msg.data = agent_response
            self.publisher.publish(response_msg)

        except Exception as e:
            self.get_logger().error(f"Error during agent processing (LangGraph invocation): {e}")
            # Implement exponential backoff for retrying requests
            self.current_interval = min(self.current_interval * 2, self.max_interval)
            error_msg_for_user = f"Error in agent processing. Retrying in {self.current_interval:.1f} seconds. Error: {e}"
            
            # Publish an error message back to the user/system
            agent_msg = String()
            agent_msg.data = error_msg_for_user
            self.publisher.publish(agent_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node) # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        pass # Allow graceful exit on Ctrl+C
    finally:
        node.destroy_node() # Clean up ROS resources
        rclpy.shutdown() # Shut down the ROS 2 system

if __name__ == "__main__":
    main()