import vosk
import sounddevice as sd
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import queue
import json

class SpeechToTextNode(Node):
    def __init__(self): 
        super().__init__('speech_to_text_node')
        self.publisher = self.create_publisher(String , 'vorker_response', 10)
        self.model = vosk.Model("/home/ken2/ros2_ws/src/ros_vlm/vosk-model-small-en-us-0.15")
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
        self.audio_queue = queue.Queue()
        self.stream  = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        )
        self.stream.start()
        self.get_logger().info('Speech to Text Node initialized. Listening for audio input.')


    def audio_callback(self, indata, frames, time, status):
        self.audio_queue.put(bytes(indata))

    def spin_once(self):
        if not self.audio_queue.empty():
            data = self.audio_queue.get()
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get('text', '')
                if text:
                    msg = String()
                    msg.data = text
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Recognized: {text}")



def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()