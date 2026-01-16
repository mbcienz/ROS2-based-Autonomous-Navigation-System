from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # acquisition node 
        Node(
            package='perception_pkg',
            executable='acquisition',
            name='acquisition_node',
            output='screen'
        ),

        # etector node
        Node(
            package='perception_pkg',
            executable='detector',
            name='detector_node',
            output='screen'
        ),

        # compute waypoint node
        Node(
            package='perception_pkg',
            executable='compute_waypoint',
            name='compute_waypoint_node',
            output='screen'
        ),
        # manager node
        Node(
            package='perception_pkg',
            executable='manager',
            name='manager_node',
            output='screen'
        ),
        # visualization node
        Node(
            package='perception_pkg',
            executable='visualization',
            name='visualization_node',
            output='screen'
        ),

    ])

