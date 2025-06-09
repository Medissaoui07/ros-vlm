import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from langchain_together import Together
import os
import time 


class AgentNode(Node): 
    def __init__(self): 
        super().__init__("agent_node")
        self.api_key = os.getenv("TOGETHER_API_KEY")


        
        self.llm = Together(
            model="deepseek-ai/DeepSeek-R1-Distill-Llama-70B-free",
            api_key=self.api_key,
            max_tokens=256,
            temperature=0.7,
            
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
            response = msg.data 
            self.get_logger().info(f"Received response from Vorker: {response}")
            prompt = f"USER: {response}\nAGENT:"
            self.get_logger().info(f"Prompt for LLM: {prompt}")
            try:
                llm_response = self.llm.invoke(prompt)
                self.get_logger().info(f"LLM response: {llm_response}")
                agent_msg = String()
                agent_msg.data = llm_response
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


