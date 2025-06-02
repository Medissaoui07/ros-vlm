from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_vlm',
            executable='cam_publisher',
            name='camera_publisher_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='ros_vlm',
            executable='image_viewer',
            name='image_viewer_node',
            output='screen'
        ),
        Node(
            package='ros_vlm',
            executable='vlm_node',
            name='vlm_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('/camera/image_raw', '/camera/image')
            ]
        )
    ])

