import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from langchain_together import ChatTogether
from langchain.schema import HumanMessage, SystemMessage
from langchain.memory import ConversationBufferMemory
from langchain.agents import Tool, AgentExecutor, initialize_agent, AgentType
from ros_vlm.utils.prompt_loader import PromptLoader 
from ros_vlm.utils.tools import vision_tool , set_global_vision_context
import os
import time 



class AgentNode(Node): 
    def __init__(self): 
        super().__init__("agent_node")
        self.api_key = os.getenv("TOGETHER_API_KEY")

        self.prompt_loader = PromptLoader()


         # Rate limiting parameters
        self.last_request_time = 0
        self.base_interval = 20  # Base interval between requests
        self.max_interval = 60   # Maximum interval
        self.current_interval = self.base_interval
        self.request_queue = []  # Queue for pending requests
        

        
        self.llm = ChatTogether(
            model="meta-llama/Llama-3.3-70B-Instruct-Turbo-Free",
            api_key=self.api_key,
            max_tokens=256,
            temperature=0.7,
            
        )

        self.memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True,
        )
        self.tools = [
            Tool(
                name="vision_tool",
                func=vision_tool, 
                description="Use this tool to get the most recent visual information available from the robot's perception. This tool takes no direct input; it will fetch the latest vision data directly."
            ),
        ]
        self.agent_chain = initialize_agent(
            tools=self.tools,
            llm=self.llm,
            agent=AgentType.CHAT_CONVERSATIONAL_REACT_DESCRIPTION,
            verbose=True, # to see the agent's reasoning
            memory=self.memory,
            handle_parsing_errors=True,  
        )

        self.create_timer(1.0, self.process_requests)



        
        self.txt_subscription= self.create_subscription(
            String, 
            "vorker_response", 
            self.response_callback, 
            10

        )
        

        self.vision_subscription = self.create_subscription(
            String, 
            "vlm_response", 
            self.vision_callback, 
            10
        )
        self.tts_publisher = self.create_publisher(
            String, 
            "agent_response", 
            10
        )
      

        
       

    def vision_callback(self, msg):   
        set_global_vision_context(msg.data)  
        self.get_logger().info(f"Updated vision Context: {msg.data}") 
        

    def response_callback(self, msg): 
        self.request_queue.append(msg.data)
        self.get_logger().info(f"Received response: {msg.data}")
    
    
    def process_requests(self):
        if not self.request_queue:
            return

        current_time = time.time()
        if current_time - self.last_request_time < self.current_interval:
            return

        user_input = self.request_queue.pop(0)
        self._run_agent(user_input)

        self.last_request_time = current_time
        self.current_interval = self.base_interval  # # Update the global vision contextReset the interval after successful processing
    def _run_agent(self , user_input):
        self.get_logger().info(f"Running agent with input: {user_input}")
        try:
            agent_response = self.agent_chain.run(
                input=user_input)
            self.get_logger().info(f"Agent response: {agent_response}")
            response_msg = String()
            response_msg.data = agent_response
            self.tts_publisher.publish(response_msg)            
        except Exception as e:
            self.get_logger().error(f"Error during agent processing: {e}")
            self.current_interval = min(self.current_interval * 2, self.max_interval)
            agent_msg = String()# Update the global vision context
            agent_msg.data = f"Error in agent processing. Retrying in {self.current_interval:.1f} seconds. Error: {e}"
            self.tts_publisher.publish(agent_msg)

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


