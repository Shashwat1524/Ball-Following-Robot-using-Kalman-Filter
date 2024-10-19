import launch
import launch_ros.actions
import os


def generate_launch_description():
    package_name = 'task_5'

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=package_name,
            executable='red_ball_tracker',
            name='red_ball_tracker',
        ),
    ])
