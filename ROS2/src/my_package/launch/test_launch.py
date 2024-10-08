from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            node_executable='odom',
            node_name='screen'
        ),
        Node(
            package='my_package',
            node_executable='ftc',
            node_name='screen'
        ),
        Node(
            package='my_package',
            node_executable='load_map',
            node_name='screen'
        ),
        Node(
            package='my_package',
            node_executable='a_star_global',
            node_name='screen'
        ),
        Node(
            package='my_package',
            node_executable='a_star_local',
            node_name='screen'
        ),
        Node(
            package='my_package',
            node_executable='test',
            node_name='screen'
        ),
        # Node(
        #     package='my_package',
        #     node_executable='mapping',
        #     node_name='screen'
        # )
    ])