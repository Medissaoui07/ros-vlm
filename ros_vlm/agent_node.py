import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from langchain_together import ChatTogether
from langchain.schema import HumanMessage , SystemMessage 
from langchain.memory import ConversationBufferMemory
import time 
import os



class AgentNode(Node): 
    def __init__(self): 
        super().__init__("agent_node")
        self.api_key = os.getenv("TOGETHER_API_KEY")


        
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
            
        
        self.subscription= self.create_subscription(
            String, 
            "vorker_response", 
            self.response_callback, 
            10

        )
        self.subscription
        self.publisher = self.create_publisher(
            String, 
            "agent_response", 
            10
        )

        self.last_request_time = 0 
        self.request_interval = 20 

    def response_callback(self , msg) : 
            now = time.time()
            if now - self.last_request_time < self.request_interval:
                self.get_logger().info("Ignoring response due to rate limiting.")
                return
            self.last_request_time = now
            response = msg.data 
            chat_hist = self.memory.chat_memory.messages
            #self.get_logger(cha).info(f"Received response from Vorker: {response}")
            messages = [ 
                SystemMessage(content="You are a helpful agent that processes responses from Vorker."),
                *chat_hist,   
                HumanMessage(content=response)
            ]
            #prompt = response + "\n\n"  
            #self.get_logger().info(f"Prompt for LLM: {prompt}")
            try:
                llm_response = self.llm.invoke(messages)
                response_txt = llm_response.content 
                self.memory.chat_memory.add_user_message(response)
                self.memory.chat_memory.add_ai_message(response_txt)
                
                self.get_logger().info(f"LLM response: {response_txt}")
                agent_msg = String()
                agent_msg.data = response_txt
                self.publisher.publish(agent_msg)
                self.get_logger().info("Published agent response.")
            except Exception as e:
                self.get_logger().error(f"Error during LLM processing: {e}")
                agent_msg = String()
                agent_msg.data = "Error processing response."
                self.publisher.publish(agent_msg)
                self.get_logger().info("Published error response.")


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


