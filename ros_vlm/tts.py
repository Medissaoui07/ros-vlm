import pyttsx3 
from rclpy.node import Node
from std_msgs.msg import String


class TTSNode(Node): 

    def __init__(self):
        super().__init__('tts_node')
       # self.publisher = self.create_publisher(String, 'tts_response', 10)
        self.subscription = self.create_subscription(
            String, 
            'agent_response', 
            self.callback, 
            10
        )
        self.engine = pyttsx3.init()
        self.engine.setProperty()
        
        self.get_logger().info("TTS Node initialized")
        
    def callback(self, msg):
        self.get_logger().info(f"Received message for TTS: {msg.data}")
        self.engine.say(msg.data)
        self.engine.runAndWait()
        #response_msg = String()
        #response_msg.data = "TTS completed"
        #self.publisher.publish(response_msg)

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()        