from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='safety_package',
        #     node_namespace='talker',
        #     node_executable='talker',
        #     node_name='talker'
        # ),
        # Node(
        #     package='safety_package',
        #     node_namespace='listner',
        #     node_executable='listner',
        #     node_name='listner'
        # ),
        Node(
            package='safety_package',
            node_executable='odom',
            node_name='odom'
        ),
        Node(
            package='safety_package',
            node_executable='ftc',
            node_name='follow_the_carrot'
        ),
        Node(
            package='safety_package',
            node_executable='load_map',
            node_name='load_map'
        ),
        Node(
            package='safety_package',
            node_executable='a_star_global',
            node_name='a_star_global'
        ),
        Node(
            package='safety_package',
            node_executable='calibration',
            node_name='calibration'
        ),
        Node(
            package='safety_package',
            node_executable='a_star_local',
            node_name='a_star_local'
        ),
        # Node(
        #     package = 'safety_package',
        #     node_executable='dwa',
        #     node_name='dwa'
        # )
        # Node(
        #     package='safety_package',
        #     node_executable='mapping',
        #     node_name='screen'
        # )
    ])