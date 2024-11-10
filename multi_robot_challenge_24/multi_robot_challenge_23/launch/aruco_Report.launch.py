import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='multi_robot_challenge_23',
            executable='marker_report',
            namespace='tb3_0',
            name='marker_report_tb3_0',
            parameters=[{"namespace": "tb3_0"}],  # Set namespace parameter explicitly
        ),
        Node(
            package='multi_robot_challenge_23',
            executable='marker_report',
            namespace='tb3_1',
            name='marker_report_tb3_1',
            parameters=[{"namespace": "tb3_1"}],  # Set namespace parameter explicitly
        ),
        Node(
            package='multi_robot_challenge_23',
            executable='explorer',
            namespace='tb3_0',
            name='marker_report_tb3_1',
            parameters=[{"namespace": "tb3_0"}],
        ),
        Node(
            package='multi_robot_challenge_23',
            executable='explorer',
            namespace='tb3_1',
            name='marker_report_tb3_1',
            parameters=[{"namespace": "tb3_1"}],
        ),
        Node(
            package='scoring',
            executable='scoring',
            name='scoring'
        ),

    ])
