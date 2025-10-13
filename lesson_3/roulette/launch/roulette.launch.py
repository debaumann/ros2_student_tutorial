import launch_ros.actions
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='roulette',
            executable='roulette_server',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='roulette',
            executable='roulette_client',
            output='screen',
            prefix='xterm -e'  # Open client in a new terminal window
        ),
    ])