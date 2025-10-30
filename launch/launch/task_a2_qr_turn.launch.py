# ~/ros2_ws/src/tb3_final_exam/launch/task_a2_qr_turn.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    exam_params = os.path.join(
        get_package_share_directory('tb3_final_exam'),
        'config', 'exam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='tb3_final_exam',
            executable='task_a2_qr_turn',
            name='task_a2_qr_turn',
            parameters=[exam_params],
            output='screen'
        )
    ])

