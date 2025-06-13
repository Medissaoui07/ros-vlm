import vosk
import sounddevice as sd
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String , Bool
import queue
import json

class SpeechToTextNode(Node):
    def __init__(self): 
        super().__init__('speech_to_text_node')
        self.publisher = self.create_publisher(String , 'vorker_response', 10)
        self.tts_status_sub = self.create_subscription(
            Bool, 
            'tts_status', 
            self.tts_status_callback, 
            10
        )
        self.is_robot_speaking = False
        self.model = vosk.Model("/home/ken2/ros2_ws/src/ros_vlm/vosk-model-small-en-us-0.15")
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
        self.audio_queue = queue.Queue()
        self.stream  = None 
        self.audio_stream()
        
        self.stream.start()
        self.get_logger().info('Speech to Text Node initialized. Listening for audio input.')


    def audio_callback(self, indata, frames, time, status):
        if not self.is_robot_speaking:
            self.audio_queue.put(bytes(indata))

    def spin_once(self):
        if not self.is_robot_speaking and not self.audio_queue.empty():
            data = self.audio_queue.get()
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get('text', '')
                if text:
                    msg = String()
                    msg.data = text
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Recognized: {text}")
    def audio_stream(self):
        if self.stream is None or (hasattr(self.stream, 'active') and not self.stream.active):
            self.stream = sd.InputStream(
                samplerate=16000, 
                blocksize=8000, 
                dtype='int16', 
                channels=1, 
                callback=self.audio_callback
            )
            self.stream.start()
    def stop_audio_stream(self):
        if self.stream is not None and self.stream.active:
            self.stream.stop()
            self.stream.close()
            #self.stream = None
            self.get_logger().info("Audio stream stopped.")
        else:
            self.get_logger().info("Audio stream is already stopped or not initialized.")        

    def tts_status_callback(self, msg):
        self.is_robot_speaking = msg.data
        if self.is_robot_speaking:
            self.get_logger().info("Robot is currently speaking, pausing speech recognition.")
            self.stop_audio_stream()        
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()
        else:
            self.get_logger().info("Robot is not speaking, resuming speech recognition.")
            self.audio_stream()

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