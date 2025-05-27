from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        #Узел управления с клавиатуры
        Node(
            package='robot_teleop',
            executable='robot_teleop_node',
            name='keyboard_teleop',
            output='screen',
            emulate_tty=True  # Для корректного отображения нажатий клавиш
        ),

        # Узел одометрии с параметрами платформы
        # Node(
        #     package='robot_odometry',
        #     executable='robot_odometry_node',
        #     name='robot_odometry',
        #     output='screen',
        #     parameters=[{
        #         'wheel_radius': 0.05,
        #         'wheel_width': 0.02,
        #         'wheel_base': 0.26,
        #         'ticks_per_revolution': 400.0
        #     }]
        # ),

        Node(
                 package="robot_camera_processor",
                 executable="robot_camera_processor_node",
                 name="robot_camera_processor",
                 output="screen"
             ),
             Node(
                 package='tf2_ros',
                 executable='static_transform_publisher',
                 arguments=[
                     '--x', '0.25',
                     '--y', '0.0',
                     '--z', '0.2',
                     '--yaw', '1.57',
                     '--pitch', '3.14',
                     '--roll', '1.57',
                     '--frame-id', '/base_link',
                     '--child-frame-id', '/camera_link'
                 ]

             )
             ,
             Node(
                 package='tf2_ros',
                 executable='static_transform_publisher',
                 arguments=[
                 '--x', '0.25',
                 '--y', '0.0',
                 '--z', '0.2',
                 '--yaw', '0',
                 '--pitch', '0',
                 '--roll', '0',
                     '--frame-id', 'base_link',
                     '--child-frame-id', 'camera_optical_link'
                 ]
             )
             # ,
             # # Запуск RViz2
             # Node(
             #     package='rviz2',
             #     executable='rviz2',
             #     name='rviz2',
             #     arguments=['-d', 'config/aruco.rviz'],
             #     output='screen'
             # )


    ])
