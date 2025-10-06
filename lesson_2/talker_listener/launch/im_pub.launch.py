from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    data_path = get_package_share_directory('talker_listener') + '/data'
    return LaunchDescription([
        Node(
            package='talker_listener',
            executable='im_pub',
            name='im_pub',
            output='screen',
            parameters=[{'data_path': data_path}]
        ),
    ])