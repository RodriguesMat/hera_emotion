import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hera_emotion')
    web_dir = os.path.join(pkg_share, 'include', 'hera_emotion')

    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'address': '0.0.0.0',
                'port': 9090
            }]
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=web_dir, 
            output='screen'
        ),
        Node(
            package='hera_emotion',
            executable='setEmotion.py',
            name='emotion_node',
            output='screen'
        )
    ])
