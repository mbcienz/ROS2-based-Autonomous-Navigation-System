from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodo per convertire i punti in pose
        Node(
            package='navigation_pkg',
            executable='point_transformer',
            name='point_transformer_node',
            output='screen'
        ),

        # Nodo di navigazione
        Node(
            package='navigation_pkg',
            executable='waypoint_nav',
            name='waypoint_nav_node',
            output='screen'
        ),
    ])

