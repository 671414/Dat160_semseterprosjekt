import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='explorer',
            namespace='tb3_0',
            name='explorer'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='explorer',
            namespace='tb3_1',
            name='explorer'),
        
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
    ])
#