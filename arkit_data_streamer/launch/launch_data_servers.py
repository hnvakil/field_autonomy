from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arkit_data_streamer',
            namespace='pose_server',
            executable='pose_server',
            name='pose',
            parameters=[
                {'port': 35601}
            ]
        ),
        Node(
            package='arkit_data_streamer',
            namespace='image_server',
            executable='image_server',
            name='image',
            parameters=[
                {'port': 35602}
            ]
        )        
    ])
