from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        #camera publisher node
        Node(
            package='ros_vlm',
            executable='cam_publisher',
            name='camera_publisher_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        #vlm node 
        Node(
            package = 'ros_vlm',
            executable = 'vlm_node',
            name = 'vlm_node',
            output = 'screen',
            parameters = [{'use_sim_time': False}], 

        )

        #speech to text node
        ,Node(
            package='ros_vlm',
            executable='speech_to_text',
            name='speech_to_text_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        #agent node 
        Node(
            package='ros_vlm',
            executable='agent_node',
            name='agent_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            
        ),
        Node(
            package='ros_vlm',
            executable='tts',
            name='tts_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            
        )
    ])
