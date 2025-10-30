from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('tb3_final_exam'),'config','exam_params.yaml')
    return LaunchDescription([
        Node(
            package='tb3_final_exam',
            executable='exam_all',
            name='exam_all',
            parameters=[params],
            output='screen'
        )
    ])
