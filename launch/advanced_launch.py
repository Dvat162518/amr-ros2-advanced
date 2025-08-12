from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode  # Correct import for lifecycle nodes

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='advanced_nodes',
            executable='topic_publisher',
            name='topic_publisher'
        ),
        Node(
            package='advanced_nodes',
            executable='topic_subscriber',
            name='topic_subscriber'
        ),
        Node(
            package='advanced_nodes',
            executable='service_server',
            name='service_server'
        ),
        Node(
            package='advanced_nodes',
            executable='service_client',
            name='service_client'
        ),
        Node(
            package='advanced_nodes',
            executable='action_server',
            name='action_server'
        ),
        Node(
            package='advanced_nodes',
            executable='action_client',
            name='action_client'
        ),
        LifecycleNode(
            package='advanced_nodes',
            executable='lifecycle_node',
            name='lifecycle_amr_node',
            namespace='/amr'
        )
    ])
