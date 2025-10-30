from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load shared params
    pkg_share = get_package_share_directory('tb3_final_exam')
    params = os.path.join(pkg_share, 'config', 'exam_params.yaml')

    return LaunchDescription([
        Node(
            package='tb3_final_exam',
            executable='task_a1_path_follow',
            name='task_a1_path_follow',
            parameters=[params],
            output='screen',
        )
    ])
