

import os
#sudo chmod 777 /dev/input/by-path/platform-i8042-serio-0-event-kbd

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="robot_odometry",
            executable="robot_odometry_node",
            name="robot_odometry_node",
            output="screen",
            emulate_tty=True,
        ),


        Node(
            package="move_to_point_client",
            executable="move_to_point_client_node",
            name="move_to_point_client_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
                    package='robot_teleop',
                    executable='robot_teleop_node',
                    name='keyboard_teleop',
                    output='screen',
                    emulate_tty=True  # Для корректного отображения нажатий клавиш
                )

])
