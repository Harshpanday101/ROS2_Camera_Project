from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{'video_device': '/dev/video0'}],
        ),
        Node(
            package='image_conversion',
            executable='image_conversion_node',
            name='image_conversion_node',
            output='screen',
            parameters=[
                {'input_topic': '/camera/image_raw'},
                {'output_topic': '/image_converted'}
            ]
        ),
    ])
