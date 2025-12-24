import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool # Import Bool for status messages
from gtts import gTTS
import os
from pydub import AudioSegment
from pydub.playback import play
import tempfile # For robust temporary file handling

class TTSNode(Node): 

    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String, 
            'agent_response', 
            self.callback, 
            10
        )
        # Publisher for TTS status
        self.status_publisher = self.create_publisher(Bool, 'tts_status', 10)
        self.lang = 'en' 
        self.get_logger().info("gTTS Node initialized")
        
    def callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received message for gTTS: {text}")

        # 1. Publish 'True' indicating speech is about to start
        status_msg = Bool()
        status_msg.data = True
        self.status_publisher.publish(status_msg)
        self.get_logger().info("Published TTS_SPEAKING_STATUS: True")

        try:
            tts = gTTS(text=text, lang=self.lang, slow=False)
            
            # Use tempfile for unique and safer temporary files
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as fp:
                audio_file_path = fp.name
                tts.save(audio_file_path)
            self.get_logger().info(f"Audio saved to {audio_file_path}")

            audio = AudioSegment.from_mp3(audio_file_path)
            play(audio)
            self.get_logger().info("Audio playback completed.")

            os.remove(audio_file_path) # Clean up temp file

        except Exception as e:
            self.get_logger().error(f"Error during gTTS processing or playback: {e}")
        finally:
            # 2. Publish 'False' indicating speech has finished (even if an error occurred)
            status_msg.data = False
            self.status_publisher.publish(status_msg)
            self.get_logger().info("Published TTS_SPEAKING_STATUS: False")

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()