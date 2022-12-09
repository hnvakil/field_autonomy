from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arkit_data_streamer',
            executable='pose_server',
            name='pose_server',
            parameters=[
                {'port': 35601}
            ]
        ),
        Node(
            package='arkit_data_streamer',
            executable='image_server',
            name='image_server',
            parameters=[
                {'port': 35602}
            ]
        ),
        # Node(
        #     package='arkit_data_streamer',
        #     executable='gps_server',
        #     name='gps_server',
        #     parameters=[
        #         {'port': 35603}
        #     ]
        # ),        
        Node(
            package='vel_calculator',
            executable='vel_calculator',
            name='vel_calculator'
        ),
        Node(
            package='trail_finding',
            executable='find_trail',
            name='find_trail'
        )
    ])
