from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lab4_image_receiver",
            executable="lab4_image_receiver_node",
            name="lab_image_receiver_node",
            output="screen"
        ),
        Node(
            package="lab4_image_sender",
            executable="lab4_image_sender_node",
            name="lab_image_sender_node",
            output="screen"
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
            '--x', '0.25',
            '--y', '0.0',
            '--z', '0.2',
            '--yaw', '0.0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', '/base_link',
            '--child-frame-id', '/camera_link' ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '90',
            '--roll', '0',
            '--frame-id', '/camera_link',
            '--child-frame-id', '/camera_optical' ]
        )
    ])
