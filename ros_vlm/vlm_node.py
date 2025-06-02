import groq
import rclpy 
import cv2
from rclpy.node import Node 
import rclpy.qos
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge, CvBridgeError
import base64 
import os


class VLMNode(Node): 
    def __init__(self): 
        super().__init__("vlm_node")

        self.groq_api_key = os.getenv("GROQ_API_KEY")
        if not self.groq_api_key:
            raise ValueError("GROQ_API_KEY environment variable is not set.")
        self.groq_client = groq.Client(api_key=self.groq_api_key)
        self.model  = "meta-llama/llama-4-scout-17b-16e-instruct" 

        self.subscription = self.create_subscription(
            Image, 
            "camera/image", 
            self.image_callback, 
            10
        )
        self.subscription

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            String , 
            "vlm_response", 
            10 )

        self.target_image_width = 320
        self.target_image_height = 240
        self.response = "No response yet."


    def image_callback(self , msg ) : 
        self.get_logger().info("Received image from camera.")     

        try :
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #display_image = image.copy()
            #scale the image to the target size
            image = cv2.resize(image, (self.target_image_width, self.target_image_height))
            _, buffer = cv2.imencode('.jpg', image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info("Image successfully encoded to base64.")
            prompt = ("SYSTEM:"
                        "You are VisionBot, a language–vision assistant on a mobile robot."  
                        "You receive a single RGB image from the front camera (`/camera/image_raw`)."

                        "USER TASK:"
                        "“Describe what you see directly ahead, infer the type of environment (e.g., corridor, desk area, kitchen), and suggest one next action the robot could take.” "

                        "OUTPUT (plain text):"
                        "1. Scene summary: “…”"
                        "2. Environment type: “…”"
                        "3. Suggested action: “…”"

                        "No extra text." )
            
            message = [
                {
                    "role": "system",
                    "content": prompt
                },
                {
                    "role": "user",
                    "content": image_base64
                }
            ]
            try : 
                response = self.groq_client.chat.completions.create(
                    model=self.model,
                    messages=message,
                    max_tokens=512,
                    temperature=0.7
                )
                vlm_response = response.choices[0].message.content.strip()
                self.response = vlm_response
                msg = String() 
                msg.data = vlm_response 
                self.publisher.publish(msg)



                self.get_logger().info(f"VLM Response: {vlm_response}")
            except groq.GroqError as e:
                self.get_logger().error(f"Groq API error: {e}")



            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            
        

def main(args=None): 
    rclpy.init(args=args)
    vlm_node = VLMNode()
    rclpy.spin(vlm_node)
    vlm_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()