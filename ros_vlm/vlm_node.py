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

    def image_callback(self , msg ) : 
        self.get_logger().info("Received image from camera.")     

        try :
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_image = image.copy()
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
            
        font = cv2.FONT_HERSHEY_SIMPLEX 
        font_scale = 0.5
        color = (0, 255, 0)
        thickness = 1
        x= 10 
        y = 20
        wrapped_text = self.wrap_text(self.response, self.target_image_width - 20, font, font_scale, thickness)
        display_image = self.draw_text(display_image, wrapped_text, (x, y), font, font_scale, color, thickness)
        cv2.imshow("VLM Response", display_image)
        cv2.waitKey(1)  

    def draw_text(self , img , text , org , font_face , font_scale , color , thickness=1 , line_spacing=1.2): 
        x, y = org
        for i, line in enumerate(text.split('\n')):
            text_size = cv2.getTextSize(line, font_face, font_scale, thickness)[0]
            # Draw a semi-transparent background rectangle for readability
            rect_x1 = x - 5
            rect_y1 = y + int(i * text_size[1] * line_spacing) - text_size[1] - 5
            rect_x2 = x + text_size[0] + 5
            rect_y2 = y + int(i * text_size[1] * line_spacing) + 5
            
            # Ensure rectangle stays within image bounds
            rect_x1 = max(0, rect_x1)
            rect_y1 = max(0, rect_y1)
            rect_x2 = min(img.shape[1], rect_x2)
            rect_y2 = min(img.shape[0], rect_y2)

            # Draw semi-transparent rectangle
            overlay = img.copy()
            alpha = 0.6  # Transparency factor.
            cv2.rectangle(overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (0,0,0), -1) # Black background
            img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)


            # Draw text
            cv2.putText(img, line, (x, int(y + i * text_size[1] * line_spacing)),
                        font_face, font_scale, color, thickness, cv2.LINE_AA)
        return img
    def wrap_text(self, text, max_width, font_face, font_scale, thickness=1):
        words = text.split(' ')
        wrapped_lines = []
        current_line = ''
        for word in words:
            test_line = f"{current_line} {word}".strip()
            text_size = cv2.getTextSize(test_line, font_face, font_scale, thickness)[0]
            if text_size[0] <= max_width:
                current_line = test_line
            else:
                if current_line:
                    wrapped_lines.append(current_line)
                current_line = word
        if current_line:
            wrapped_lines.append(current_line)
        return '\n'.join(wrapped_lines)
    

def main(args=None): 
    rclpy.init(args=args)
    vlm_node = VLMNode()
    rclpy.spin(vlm_node)
    vlm_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()